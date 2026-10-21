

import numpy as np
from roboticstoolbox import jtraj


# ==========================================
# 1. RECTANGULAR COLLISION BOX
# ==========================================

class CollisionBox:

    def __init__(self, name, centre, size):

        self.name = name
        self.centre = np.array(centre, dtype=float)
        self.size = np.array(size, dtype=float)

        self.minimum = self.centre - self.size / 2
        self.maximum = self.centre + self.size / 2


# ==========================================
# 2. CHECK LINE AGAINST BOX
# ==========================================

def segment_intersects_box(
    point1, point2, box, margin=0.0
):

    p1 = np.asarray(point1, dtype=float)
    p2 = np.asarray(point2, dtype=float)

    direction = p2 - p1

    minimum = box.minimum - margin
    maximum = box.maximum + margin

    t_min = 0.0
    t_max = 1.0

    for axis in range(3):

        if abs(direction[axis]) < 1e-10:

            if (
                p1[axis] < minimum[axis]
                or p1[axis] > maximum[axis]
            ):
                return False

        else:

            t1 = (
                minimum[axis] - p1[axis]
            ) / direction[axis]

            t2 = (
                maximum[axis] - p1[axis]
            ) / direction[axis]

            t_enter = min(t1, t2)
            t_exit = max(t1, t2)

            t_min = max(t_min, t_enter)
            t_max = min(t_max, t_exit)

            if t_min > t_max:
                return False

    return True


# ==========================================
# 3. GET ROBOT LINK POSITIONS
# ==========================================

def get_link_positions(robot, q):

    positions = []

    T = robot.base.A.copy()

    positions.append(T[:3, 3].copy())

    for link, joint_angle in zip(robot.links, q):

        T = T @ link.A(joint_angle).A

        positions.append(T[:3, 3].copy())

    return positions


# ==========================================
# 4. CHECK ARM COLLISION
# ==========================================

def check_arm_collision(
    robot, q, obstacles,
    margin=0.02,
    verbose=False
):

    positions = get_link_positions(robot, q)

    for i in range(len(positions) - 1):

        p1 = positions[i]
        p2 = positions[i + 1]

        for obstacle in obstacles:

            if segment_intersects_box(
                p1, p2, obstacle, margin
            ):

                if verbose:
                    print("\nARM COLLISION DETECTED")
                    print("Link:", i + 1)
                    print("Obstacle:", obstacle.name)
                    print("Link start:", np.round(p1, 3))
                    print("Link end:", np.round(p2, 3))
                    print(
                        "Obstacle minimum:",
                        np.round(obstacle.minimum, 3)
                    )
                    print(
                        "Obstacle maximum:",
                        np.round(obstacle.maximum, 3)
                    )

                return True, obstacle.name

    return False, None


# ==========================================
# 5. CHECK COMPLETE ARM TRAJECTORY
# ==========================================

def check_arm_trajectory(
    robot, q_start, q_goal,
    obstacles, steps=60
):

    trajectory = jtraj(
        q_start, q_goal, steps
    ).q

    for i, q in enumerate(trajectory):

        collision, obstacle_name = (
            check_arm_collision(
                robot, q, obstacles,
                verbose=True
            )
        )

        if collision:

            print(
                f"Arm collision at step {i}: "
                f"{obstacle_name}"
            )

            return False

    return True


# ==========================================
# 6. CHECK TURTLEBOT PATH
# ==========================================

def check_turtlebot_path(
    start, finish, obstacles,
    robot_radius=0.18,
    steps=100
):

    start = np.asarray(start, dtype=float)
    finish = np.asarray(finish, dtype=float)

    for value in np.linspace(0, 1, steps):

        position = start + value * (finish - start)

        x, y = position[:2]

        for obstacle in obstacles:

            minimum = (
                obstacle.minimum[:2] - robot_radius
            )

            maximum = (
                obstacle.maximum[:2] + robot_radius
            )

            if (
                minimum[0] <= x <= maximum[0]
                and minimum[1] <= y <= maximum[1]
            ):

                print(
                    "TurtleBot collision detected:",
                    obstacle.name
                )

                return False

    return True
