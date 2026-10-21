"""Manual controls for the robot objects in the running shared workcell."""
import numpy as np
from spatialmath import SE3
from Collective_Folder import workcell_actions


class RobotBridge:
    def __init__(self):
        self.env = None
        self.robots = {}
        self.runtime = None
        self.draw = None
        self.validators = {}
        self.saved = {}

    def attach(self, env, robots, runtime, draw, validators=None):
        self.env, self.robots, self.runtime, self.draw = env, robots, runtime, draw
        self.validators = validators or {}
        runtime.bridge = self

    def load(self, name):
        if name not in self.robots:
            raise ValueError("Wait until the shared workcell is fully loaded")
        return self.robots[name]

    def begin_manual(self, name):
        robot = self.load(name)
        if self.runtime.state not in ('READY', 'PAUSED', 'COMPLETE'):
            raise ValueError('Pause automatic motion before jogging')
        if name not in self.saved:
            self.saved[name] = (np.asarray(robot.q, float).copy(), SE3(robot.base))

    def limits_deg(self, name):
        robot = self.load(name)
        limits = np.asarray(robot.qlim, float)
        if limits.shape != (2, robot.n) or not np.all(np.isfinite(limits)):
            limits = np.array([[-np.pi] * robot.n, [np.pi] * robot.n])
            source = 'Provisional simulation limits; confirm with model author'
        else:
            source = 'Robot model joint limits'
        if hasattr(robot, 'safe_qlim'):
            limits = np.array([np.maximum(limits[0], robot.safe_qlim[0]),
                               np.minimum(limits[1], robot.safe_qlim[1])])
        return np.rad2deg(limits), source

    def angles_deg(self, name):
        return np.rad2deg(np.asarray(self.load(name).q, float))

    def tool_pose(self, name, q=None):
        robot = self.load(name)
        q = robot.q if q is None else q
        if name == 'ReBot':
            links = robot.get_link_transforms(q)
            return robot.base * robot.get_gripper_transforms(links, robot.gripper_opening)[0]
        if name == 'HannahBot':
            return robot.measured_fkine(q)
        return robot.fkine(q)

    def _valid(self, name, q):
        robot = self.load(name)
        limits, _ = self.limits_deg(name)
        if q.shape != (robot.n,) or not np.all(np.isfinite(q)):
            raise ValueError('Expected finite joint angles for this robot')
        angles = np.rad2deg(q)
        if np.any(angles < limits[0]) or np.any(angles > limits[1]):
            raise ValueError('Outside configured joint limits')
        validator = self.validators.get(name)
        if validator is not None and not validator(q):
            raise ValueError('Existing collision/limit guard rejected the motion')
        if not workcell_actions.validate_manual(self.runtime, name, q):
            raise ValueError('Team safety module rejected the motion')
        if not self.runtime.configuration_clear(name, q):
            raise ValueError('Registered team collision check rejected the motion')

    def move_to(self, name, target):
        self.begin_manual(name)
        robot = self.load(name)
        start = np.asarray(robot.q, float).copy()
        target = np.asarray(target, float)
        count = max(2, int(np.ceil(np.max(np.abs(target-start)) / np.deg2rad(1))))
        path = np.linspace(start, target, count + 1)[1:]
        # Validate the entire sampled path before changing the robot.
        for q in path:
            self._valid(name, q)
        for q in path:
            if self.runtime.closed or self.runtime.state == 'ESTOP':
                raise ValueError('Manual motion interrupted by stop')
            self._valid(name, q)  # Scene hazards may change after preflight.
            robot.q = q.copy()
            if hasattr(robot, 'update_meshes'):
                robot.update_meshes()
            self.draw(0.025)
        self.runtime.message = f'{name}: joint pose updated in the shared scene'

    def set_joint(self, name, index, angle_deg):
        robot = self.load(name)
        if not 0 <= index < robot.n or not np.isfinite(angle_deg):
            raise ValueError('Invalid joint command')
        q = np.asarray(robot.q, float).copy()
        q[index] = np.deg2rad(angle_deg)
        self.move_to(name, q)

    def translate(self, name, axis, distance):
        self.begin_manual(name)
        robot = self.load(name)
        q = np.asarray(robot.q, float).copy()
        initial = self.tool_pose(name, q)
        goal = initial.t.copy()
        goal[axis] += distance
        # Numerical translational Jacobian also supports the custom ReBot model.
        # Retain orientation using spatial rotation error and damped least squares.
        def rotation_error(target_R, current_R):
            delta = target_R @ current_R.T
            # Local spatial orientation error; each requested Cartesian jog is small.
            return 0.5 * np.array([delta[2,1]-delta[1,2],
                                   delta[0,2]-delta[2,0], delta[1,0]-delta[0,1]])
        for _ in range(150):
            pose = self.tool_pose(name, q)
            error = np.r_[goal-pose.t, rotation_error(initial.R, pose.R)]
            if np.linalg.norm(error[:3]) < 0.001 and np.linalg.norm(error[3:]) < 0.01:
                self.move_to(name, q)
                return
            jac = np.zeros((6, robot.n))
            for j in range(robot.n):
                dq = np.zeros(robot.n); dq[j] = 1e-5
                shifted = self.tool_pose(name, q+dq)
                jac[:3, j] = (shifted.t-pose.t) / 1e-5
                jac[3:, j] = rotation_error(shifted.R, pose.R) / 1e-5
            step = jac.T @ np.linalg.solve(jac @ jac.T + 0.02**2*np.eye(6), error)
            q += np.clip(step, -0.08, 0.08)
            limits, _ = self.limits_deg(name)
            q = np.clip(q, *np.deg2rad(limits))
        raise ValueError('Cartesian target unreachable with the current orientation')

    def restore(self, name):
        if name not in self.saved:
            return
        q, base = self.saved[name]
        robot = self.load(name)
        # Base is saved for correspondence with the automatic trajectory.
        if not np.allclose(robot.base.A, base.A):
            raise ValueError('Robot base changed; teammate recovery is required')
        self.move_to(name, q)
        del self.saved[name]
        self.runtime.message = f'{name}: restored to its pose before manual control'

    def restore_all(self):
        for name in list(self.saved):
            self.restore(name)

    def home(self, name):
        self.begin_manual(name)
        self.runtime.home(name)
