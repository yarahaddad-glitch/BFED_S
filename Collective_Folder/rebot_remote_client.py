#!/usr/bin/env python3
"""
rebot_remote_client.py - remote-side control of the reBot B601-DM over rosbridge (roslibpy).
Equivalent of new_ur_control_script.py + simple_gripper_control.py, with the gripper as joint 7.

    pip install roslibpy
    python rebot_remote_client.py --host rebot-pi.local                       # read-only: joints + tool pose
    python rebot_remote_client.py --joints 0.3 -0.8 -0.9 0.2 0.1 0.0 --duration 5
    python rebot_remote_client.py --pose 0.25 0.0 0.25 0 0 0 --duration 5     # x y z (m), roll pitch yaw (rad)
    python rebot_remote_client.py --joints ... --pose ...                     # joints first, then the pose

Joint order: joint1..joint6 + gripper, all in rad (gripper: 0.0 closed, -5.0 open on the DM build).
Limits are enforced on the Pi; a rejected command returns "failed: <joint> ... outside limits".
"""
import argparse
import math
import threading
import time

import roslibpy

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
NS = "/rebot"
GRIPPER_OPEN, GRIPPER_CLOSED = -5.0, 0.0      # rad, must match the Pi's launch file


def rpy_to_quat(roll, pitch, yaw):
    """Fixed-axis roll/pitch/yaw (R = Rz*Ry*Rx, the ROS convention) to (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class RebotClient:
    def __init__(self, host, port=9090):
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run(timeout=10)                       # call run() once; it blocks until connected
        if not self.ros.is_connected:
            raise RuntimeError(f"Could not reach rosbridge at {host}:{port}")

        self._state = None
        self._pose = None
        self._status = None
        self._status_event = threading.Event()
        self._lock = threading.Lock()

        # ROS 2 type names use the /msg/ and /srv/ infix
        self._js = roslibpy.Topic(self.ros, f"{NS}/joint_states", "sensor_msgs/msg/JointState",
                                  throttle_rate=50)
        self._js.subscribe(self._on_state)
        self._tp = roslibpy.Topic(self.ros, f"{NS}/tool_pose", "geometry_msgs/msg/PoseStamped",
                                  throttle_rate=100)
        self._tp.subscribe(self._on_pose)
        self._st = roslibpy.Topic(self.ros, f"{NS}/execution_status", "std_msgs/msg/String")
        self._st.subscribe(self._on_status)
        self._cmd = roslibpy.Topic(self.ros, f"{NS}/joint_trajectory_controller/command",
                                   "trajectory_msgs/msg/JointTrajectory")
        self._cmd.advertise()

    # ------------------------------------------------------------- state
    def _on_state(self, msg):
        # Reorder by name instead of assuming an index order (replaces the UR swap of joints 0 and 2)
        pos = dict(zip(msg["name"], msg["position"]))
        with self._lock:
            self._state = [pos[j] for j in JOINTS] if all(j in pos for j in JOINTS) else None

    def _on_pose(self, msg):
        p, o = msg["pose"]["position"], msg["pose"]["orientation"]
        with self._lock:
            self._pose = (p["x"], p["y"], p["z"]) + quat_to_rpy(o["x"], o["y"], o["z"], o["w"])

    def tool_pose(self, timeout=5.0):
        """Current end_link pose in base_link: (x, y, z) in m, (roll, pitch, yaw) in rad."""
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if self._pose is not None:
                    return list(self._pose)
            time.sleep(0.05)
        raise RuntimeError(f"No pose on {NS}/tool_pose (is kinematics loaded on the Pi?)")

    def _on_status(self, msg):
        self._status = msg["data"]
        if self._status != "executing":
            self._status_event.set()

    def joint_positions(self, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if self._state is not None:
                    return list(self._state)
            time.sleep(0.05)
        raise RuntimeError(f"No joint state on {NS}/joint_states")

    # ------------------------------------------------------------- motion
    def send_trajectory(self, points, joint_names=JOINTS, wait=True, timeout=None):
        """points: list of (time_from_start_s, positions). Gripper value in a point = gripper angle (rad) at that waypoint."""
        msg = {
            "joint_names": list(joint_names),
            "points": [{
                "positions": [float(p) for p in pos],
                # ROS 2 builtin_interfaces/Duration: sec / nanosec (ROS 1 used secs / nsecs)
                "time_from_start": {"sec": int(t), "nanosec": int(round((t - int(t)) * 1e9))},
            } for t, pos in points],
        }
        self._status_event.clear()
        self._cmd.publish(roslibpy.Message(msg))
        if not wait:
            return None
        total = (timeout or points[-1][0] + 15.0)
        if not self._status_event.wait(total):
            raise TimeoutError("No result on execution_status")
        return self._status

    def move_joints(self, target, duration=5.0, wait=True):
        """target: 6 arm positions, or 7 including the gripper angle (rad)."""
        names = JOINTS if len(target) == 7 else JOINTS[:6]
        return self.send_trajectory([(duration, target)], names, wait)

    def set_gripper(self, position, wait=True):
        """Gripper as joint 7 on its own, in rad (GRIPPER_CLOSED .. GRIPPER_OPEN)."""
        return self.send_trajectory([(0.0, [position])], ["gripper"], wait)

    def solve_ik(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """Joint angles (6) for an end_link pose in base_link. Solves on the Pi; never moves."""
        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)
        srv = roslibpy.Service(self.ros, f"{NS}/solve_ik", "rebotarm_msgs/srv/MoveToPoseIK")
        res = srv.call(roslibpy.ServiceRequest({"target_pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}}}), timeout=30)
        print(f"[ROS] solve_ik: {res['message']}")
        if not res["success"]:
            raise RuntimeError(res["message"])
        return list(res["q_solution"])

    def move_to_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0, duration=5.0, gripper=None, wait=True):
        """IK on the Pi, then an ordinary joint trajectory (same limit/velocity checks)."""
        q = self.solve_ik(x, y, z, roll, pitch, yaw)
        return self.move_joints(q + ([gripper] if gripper is not None else []), duration, wait)

    # ------------------------------------------------------------- services (std_srvs/Trigger)
    def call(self, name, timeout=30):
        srv = roslibpy.Service(self.ros, f"{NS}/{name}", "std_srvs/srv/Trigger")
        res = srv.call(roslibpy.ServiceRequest({}), timeout=timeout)
        print(f"[ROS] {name}: success={res['success']} message='{res['message']}'")
        return res["success"]

    def enable(self):     return self.call("enable")
    def disable(self):    return self.call("disable")      # motors go limp: support the arm first
    def safe_home(self):  return self.call("safe_home", timeout=60)
    def stop(self):       return self.call("stop")
    def open_gripper(self):  return self.call("gripper/open")
    def close_gripper(self): return self.call("gripper/close")

    def close(self):
        self._cmd.unadvertise()
        self._js.unsubscribe()
        self._tp.unsubscribe()
        self._st.unsubscribe()
        self.ros.terminate()


def main():
    ap = argparse.ArgumentParser(description="Move the reBot to joint angles, then to a tool pose.")
    ap.add_argument("--host", default="rebot-pi.local", help="Pi hostname or IP")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--joints", type=float, nargs="+", metavar="RAD",
                    help="6 arm angles, or 7 with the gripper (rad)")
    ap.add_argument("--pose", type=float, nargs=6, metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
                    help="end_link pose in base_link: m and rad")
    ap.add_argument("--duration", type=float, default=5.0, help="seconds per move")
    args = ap.parse_args()
    if args.joints is not None and len(args.joints) not in (6, 7):
        ap.error("--joints takes 6 or 7 values")

    arm = RebotClient(args.host, args.port)
    try:
        print(f"[ROS] Joints: {[round(v, 3) for v in arm.joint_positions()]}")
        print(f"[ROS] Tool pose (x y z r p y): {[round(v, 3) for v in arm.tool_pose()]}")
        if args.joints is None and args.pose is None:
            return                                      # read-only run
        if not arm.enable():
            raise RuntimeError("enable failed")
        if args.joints is not None:
            print("[APP] Joint move:", arm.move_joints(args.joints, args.duration))
        if args.pose is not None:
            print("[APP] Pose move:", arm.move_to_pose(*args.pose, duration=args.duration))
        print(f"[ROS] Tool pose now: {[round(v, 3) for v in arm.tool_pose()]}")
    except KeyboardInterrupt:
        print("\n[APP] Interrupted, stopping arm.")
        arm.stop()
    finally:
        arm.close()                 # arm stays enabled and holding; call safe_home() to park + disable


if __name__ == "__main__":
    main()
