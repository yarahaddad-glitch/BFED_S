
import time
import numpy as np
import swift

from Rebot_class import ReBot
from rebot_remote_client import RebotClient


def main():
    # Create the Swift simulation
    env = swift.Swift()
    env.launch(realtime=True)

    # Load the ReBot robot model into Swift
    robot = ReBot(env)

    # Connect to Raspberry Pi (read-only)
    client = RebotClient("rebot-pi.local")

    print("Connected to Raspberry Pi")
    print("Swift is now following the physical ReBot")

    try:
        while True:
            # Read 7 joint values from ROS
            joints = client.joint_positions()

            # First 6 values control the arm
            q = np.asarray(joints[:6], dtype=float)

            # Check joint limits before updating Swift
            if np.all(np.isfinite(q)) and np.all(
                (q >= robot.qlim[0]) & (q <= robot.qlim[1])
            ):
                robot.q = q
            else:
                print("Invalid joint readings:", q)

            # Update Swift
            env.step(0.05)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopped live ReBot visualisation")

    finally:
        client.close()


if __name__ == "__main__":
    main()
