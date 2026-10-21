"""YARA: team-owned emergency-stop and recovery integration points.

The GUI calls these on the simulation thread with a WorkcellRuntime context:
context.env, context.robots, context.objects and context.request_stop(reason).
The built-in simulation latch stops further steps independently of these hooks.
Physical inputs must call request_stop asynchronously; release must NOT reset it.
Implement/verify the physical stop, reset conditions and planned home/restart here.
This file does not provide a certified safety controller.
"""


def on_stop(context):
    """YARA: send the physical stop command here. Do not move robots home."""
    return "Simulation stopped; physical emergency-stop integration pending."


def on_reset(context):
    """YARA: return False while any physical/sensor stop input remains active."""
    return True  # Simulation acknowledgement only; never starts motion.


def on_home(context, robot_name):
    """YARA: add a checked home trajectory; return True when successful.

    Use context.bridge.move_to(robot_name, q_home), or context.motion_step(dt)
    for your own checked samples. Calling env.step while paused waits for Start.
    """
    return False


def on_restart(context):
    """YARA: restore objects, queues and robots safely before a new sequence.

Return True only once a complete restart is implemented. Reset and Start are
separate actions. Do not call main() or create another Swift environment.
"""
    return False


def validate_manual(context, robot_name, q):
    """Optional extra manual-only veto for proposed joints.

Prefer install_collision_checks for checks shared with automatic motion.
Return False or raise ValueError to reject. Existing controller checks also run.
"""
    return True


def install_collision_checks(context):
    """TEAM: register each robot's pure collision predicate here after scene loading.

    Example (replace with your module and function):
        from Hannah_Work.my_checks import hannah_clear
        context.register_collision_check('HannahBot', hannah_clear)

    def hannah_clear(context, proposed_q, payload_pose):
        # Calculate geometry from proposed_q; do not move the robot or call env.step.
        # Do not call controller.valid / configuration_safe here (recursive).
        return True  # or False / a reason string when blocked

    Names: MaryamBot, HannahBot, YaraBot, ReBot. Checks are shared by automatic
    joint commands and manual controls. Mobile-base motion needs its own proposed
    base-pose check in the owner's drive routine as well.
    """
