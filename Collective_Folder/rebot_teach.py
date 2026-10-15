#!/usr/bin/env python3
"""
rebot_teach.py - move the rebot by hand, read its joint angles, and record named poses.

Keep next to rebot_remote_client.py and run:
    py rebot_teach.py                    # Windows
    python3 rebot_teach.py               # Linux

Commands at the prompt:
    <name>      record the current joint angles under that name (e.g. "hover above")
    <Enter>     just print the current angles
    live        stream angles continuously until Ctrl+C
    list        show everything recorded so far
    undo        remove the last recorded pose
    q           finish: print a TEACH dict to paste into rebot_sequence.py,
                and save it to teach_poses.json

SAFETY: with the motors disabled the arm may sag under gravity. Support it with
one hand before running, and keep it clear of people and obstacles.
"""
import json
import time

from rebot_remote_client import RebotClient

HOST = "192.168.50.2"
FREE_DRIVE = True        # True = try to disable the motors so you can move the arm by hand


def read_joints(arm):
    """6 arm joints (rad). joint_positions() has the gripper as a 7th value."""
    return [round(v, 4) for v in arm.joint_positions()[:6]]


def read_gripper(arm):
    pos = arm.joint_positions()
    return round(pos[6], 4) if len(pos) > 6 else None


def release_motors(arm):
    """Let the arm be moved by hand. Uses arm.disable() if the client has it."""
    disable = getattr(arm, "disable", None)
    if disable is None:
        print("[warn] RebotClient has no disable() method - the arm may be held stiff.")
        print("       Check rebot_remote_client.py for the call that releases the motors.")
        return False
    disable()
    print("Motors released - you can move the arm by hand now.")
    return True


def live(arm):
    print("Live readout (Ctrl+C to stop):")
    try:
        while True:
            print("  ", read_joints(arm), "grip:", read_gripper(arm), end="\r")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def dump(poses):
    print("\nTEACH = {")
    items = list(poses.items())
    for i, (name, q) in enumerate(items):
        comma = "," if i < len(items) - 1 else ""
        print(f'    "{name}": {q}{comma}')
    print("}")


def main():
    arm = RebotClient(HOST)
    poses = {}
    try:
        time.sleep(0.5)                              # let a joint_states update arrive
        print("Connected. Current joints:", read_joints(arm))
        if FREE_DRIVE:
            input("Support the arm, then press Enter to release the motors...")
            release_motors(arm)

        while True:
            cmd = input("\nname / Enter / live / list / undo / q > ").strip()
            if cmd == "q":
                break
            elif cmd == "":
                print("  joints:", read_joints(arm), " gripper:", read_gripper(arm))
            elif cmd == "live":
                live(arm)
            elif cmd == "list":
                dump(poses)
            elif cmd == "undo":
                if poses:
                    name, _ = poses.popitem()
                    print(f"  removed '{name}'")
            else:
                q = read_joints(arm)
                poses[cmd] = q
                print(f"  recorded '{cmd}': {q}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if poses:
            dump(poses)
            with open("teach_poses.json", "w") as f:
                json.dump(poses, f, indent=2)
            print("\nSaved to teach_poses.json")
        arm.close()


if __name__ == "__main__":
    main()
