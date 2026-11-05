# shared meal sequence for MaryamBot, YaraBot and ReBot
import numpy as np
from spatialmath import SE3
from spatialgeometry import Cuboid, Mesh

from Maryam_Work.maryam_movements import (
    MaryamMovements, food_geometry, mesh_xyz, TRAY_SCALE, BELT_FAR_LANE_Y)
from Maryam_Work.maryam_tray_delivery import TRAY_CENTRE_UNSCALED
from Collective_Folder import yara_main_testing as yara
from Collective_Folder.Rebot_class import ReBot
from Collective_Folder.rebot_towels import place_towel

BELT_SPEED = 0.45
YARA_STOP_X = 13.2
REBOT_STOP_X = 12.5
REBOT_BASE = (12.5, 3.6, 0.66)


# SHOW_CONVEYOR - Slide the tray along one straight belt leg; this is pose animation, not RMRC.
def carry_tray(env, tray, target, riders=(), dt=0.05, clearance=None):

    if dt <= 0 or BELT_SPEED <= 0:
        raise ValueError("Conveyor speed and timestep must be positive")
    initial = SE3(tray.T)
    target = np.asarray(target, dtype=float)
    # Save food/towel poses relative to the tray so they travel together.
    offsets = [(r, initial.inv() * SE3(r.T)) for r in riders]
    steps = max(1, int(np.ceil(np.linalg.norm(target - initial.t) / BELT_SPEED / dt)))
    # Precheck tray samples using the supplied clearance function.
    # serve() supplies tray-versus-Maryam clearance at her current joint configuration.
    if clearance is not None:
        for s in np.linspace(0, 1, steps + 1):
            pose = SE3.Rt(initial.R, initial.t + s * (target - initial.t))
            if not clearance(pose):
                raise RuntimeError("Conveyor tray would hit Maryam; belt stopped")
    for s in np.linspace(0, 1, steps + 1)[1:]:
        pose = SE3.Rt(initial.R, initial.t + s * (target - initial.t))
        tray.T = pose.A
        for rider, relative in offsets:
            rider.T = (pose * relative).A
        env.step(dt)


# Follow a sequence of relative X/Y conveyor legs, carrying the supplied riders.
def carry_tray_route(env, tray, route, riders=(), clearance=None):
    """Follow relative (dx, dy) legs as in the supplied conveyor code."""
    for dx, dy in route:
        carry_tray(env, tray, SE3(tray.T).t + [dx, dy, 0], riders, clearance=clearance)


# Reuse Maryam's pickup and handout methods; replace the parent's serve().
class IntegratedMeal(MaryamMovements):
    # Build shared Maryam components, Yara, ReBot, food geometry and towel inventory.
    def __init__(self, env, objects, robot, trays, activity=None):
        self.activity = activity or (lambda message: None)
        # Parent setup creates the controller, layout, tray stacks and TrayDelivery.
        super().__init__(env, objects, robot, trays)
        yara.setup_yarabot(env, objects)
        self.food_bottom = {}
        for colour in ("blue", "green"):
            path, scale, bottom = food_geometry(colour)
            self.food_bottom[colour] = bottom
            for name in yara.FOOD_NAMES[colour]:
                # Swift's live shape_update can stall waiting for the browser.
                # Configure scale before adding a replacement, as for other meshes.
                original_food = objects[name]
                food = Mesh(str(path), scale=[scale] * 3, pose=SE3(original_food.T))
                env.add(food)
                original_food.T = SE3(0, 0, -100).A
                objects[name] = food
        self.rebot = ReBot(env)
        # Wider visual opening for the folded serving towels in this simulation.
        self.rebot.GRIPPER_MAX = 0.10
        self.rebot.base = SE3(*REBOT_BASE) * SE3.Rz(np.pi / 2)
        self.rebot.update_meshes()
        # The source scene supplies only one decorative towel, not two.
        original = objects.pop("papertowel1", None)
        if original is not None:
            original.T = SE3(0, 0, -100).A
        self.towels = []
        for i in range(len(trays)):
            towel = Cuboid(scale=[0.14, 0.12, 0.006], color=(0.98, 0.98, 0.94, 1))
            # Centre counter between the conveyor lanes, beside ReBot.
            # Measured ConvCounter top; towel layers rest directly together.
            towel.T = SE3(12.8, 3.45, 0.629814 + 0.003 + i * 0.006).A
            env.add(towel)
            self.towels.append(towel)
        env.step()

    # Run one meal: Maryam pickup -> Yara food -> ReBot towel -> Maryam handout.
    def serve(self, colour, on_conveyor):
        colour = colour.lower()
        pick = self.stacks.next_tray(colour)
        if pick is None:
            print(f"No {colour} trays left.", flush=True)
            return False
        if not self.towels or not yara.FOOD_NAMES.get(colour):
            raise RuntimeError(f"No towel or {colour} food available")
        key, tray = pick
        self.activity(f'MaryamBot: picking up {colour} tray')
        # SHOW_PICKUP - Inherited Maryam pickup uses TrayDelivery's grasp IK and RMRC carrying.
        outcome = self.pickups[colour](tray, key)
        if outcome.status != "PLACED":
            raise RuntimeError(f"Pickup failed: {outcome.message}")
        self.stacks.mark_used(key)
        # Update logical order status only after successful tray deposit.
        on_conveyor()
        initial = SE3(tray.T)
        centre_offset = initial.R @ (TRAY_CENTRE_UNSCALED * TRAY_SCALE)
        start = initial.t + centre_offset
        left = mesh_xyz(self.objects, "Conv2_2", (11.92, 3.429, .58))[0]
        right = mesh_xyz(self.objects, "Conv2_1", (14.96, 3.429, .58))[0]
        far_y, near_y = BELT_FAR_LANE_Y, start[1]

        # Convert tray-centre waypoints to mesh-origin movements and call the conveyor helper.
        def travel(points, riders=()):
            # Convert centre waypoints to the teammate's relative mesh-origin legs.
            for x, y in points:
                target = np.array([x, y, start[2]]) - centre_offset
                delta = target - SE3(tray.T).t
                carry_tray_route(self.env, tray, [(delta[0], delta[1])], riders,
                                 clearance=lambda pose: self.delivery.tray_clears_robot(
                                     self.delivery.robot.q, pose))

        print("[BELT] Travelling to Yara's food station.", flush=True)
        travel([(right, near_y), (right, far_y), (YARA_STOP_X, far_y)])
        before = list(yara.FOOD_NAMES[colour])
        self.activity(f'YaraBot: placing food on {colour} tray')
        # SHOW_YARA - Integration: call Yara's food-placement task when the tray reaches her station.
        if not yara.run_task(colour, tray=tray, food_bottom_centre=self.food_bottom[colour]):
            raise RuntimeError("Yara could not complete food placement")
        # Food removed from Yara's available inventory becomes the tray's carried riders.
        riders = [self.objects[n] for n in before if n not in yara.FOOD_NAMES[colour]]
        if not riders:
            raise RuntimeError("Yara reported success without placing food")
        print("[BELT] Travelling to ReBot's towel station.", flush=True)
        travel([(left, far_y), (left, near_y), (REBOT_STOP_X, near_y)], riders)
        towel = self.towels[-1]
        drop = SE3(tray.T) * SE3(*(TRAY_CENTRE_UNSCALED * TRAY_SCALE + [0.17, 0, 0.045]))
        self.activity(f'ReBot: placing towel on {colour} tray')
        # SHOW_REBOT - Simulated towel grasp/place uses contact-point IK and joint animation.
        place_towel(self.env, self.rebot, towel, drop)
        self.towels.pop()
        riders.append(towel)
        print("[BELT] Returning loaded tray to Maryam.", flush=True)
        travel([(start[0], near_y)], riders)
        # Save food/towel attachment poses for Maryam's loaded-tray pickup.
        self.delivery.payload = [(r, SE3(tray.T).inv() * SE3(r.T)) for r in riders]
        self.activity(f'MaryamBot: handing out {colour} tray')
        # SHOW_HANDOUT - Inherited Maryam action turns/presents the meal through the serving gap.
        self.handouts[colour](tray, key)
        self.activity(f'{colour.capitalize()} tray delivered')
        return True
