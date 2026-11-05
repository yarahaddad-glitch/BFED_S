# Checked Home/return workflow. WorkcellRecovery is the all-arm implementation used by main_all.
# It retains explicit payload attachments and leaves mobile bases at their current world positions.


import numpy as np
from spatialmath import SE3
from pathlib import Path
from functools import lru_cache
from Maryam_Work.joint_planner import plan_joint_route
from Maryam_Work.maryamrobot_controller import stl_bounds
from Maryam_Work.collision_geometry import boxes_overlap


class MaryamRecovery:
    def __init__(self, runtime, movements):
        self.runtime, self.movements = runtime, movements
        self.delivery = movements.delivery
        self.return_q = None

    def _move(self, target):
        previous = self.runtime.state
        self.runtime.set_state('RECOVERING', 'Checking and executing a Maryam recovery route')
        try:
            self._animate(target)
        finally:
            # A new stop must never be overwritten by the old paused state.
            if self.runtime.state == 'RECOVERING':
                self.runtime.set_state(previous, 'Recovery complete; automation remains paused')

    def _animate(self, target):
        runtime, delivery = self.runtime, self.delivery
        controller, robot = delivery.controller, delivery.robot
        generation = runtime.stop_generation

        def pulse():
            if (runtime.closed or runtime.state != 'RECOVERING'
                    or generation != runtime.stop_generation):
                raise ValueError('Recovery interrupted by a new stop')

        route = plan_joint_route(robot.q.copy(), target, delivery.recovery_clear,
                                 controller.q_min, controller.q_max, pulse=pulse)
        if route is None:
            raise ValueError('No verified Home/recovery route; robot remains stationary')
        runtime.log('MaryamBot: checked recovery motion')
        for q in route[1:]:
            pulse()
            if not delivery.recovery_clear(q):
                runtime.request_stop('MaryamBot recovery route became blocked')
                raise ValueError('Recovery route became blocked')
            robot.q = q.copy()
            delivery.update_payload()
            runtime.motion_step(.025)

    # Maryam-only base helper; the final all-arm workflow uses WorkcellRecovery.home_robot().
    def home(self):
        runtime = self.runtime
        if runtime.safety and runtime.safety.check():
            raise ValueError('Clear the unsafe zone / light curtain before Home')
        if runtime.collision and runtime.collision.active_hazards():
            raise ValueError('Remove the collision-test obstacle before Home')
        if self.return_q is None:
            saved = runtime.bridge.saved.get('MaryamBot')
            self.return_q = (saved[0].copy() if saved is not None
                             else self.delivery.robot.q.copy())
        self._move(self.movements.home_q)
        runtime.bridge.saved.pop('MaryamBot', None)
        runtime.message = 'MaryamBot home. Automation paused; Start restores the task pose first.'

    def restore(self):
        if self.return_q is not None:
            self._move(self.return_q)
            self.return_q = None


# Cache a conservative mesh bounding box and its local centre for recovery collision checks.
@lru_cache(maxsize=100)
def mesh_box(filename, scale):
    """Local mesh bounds; COLLADA node/unit transforms are included."""
    if Path(filename).suffix.lower() == '.stl':
        lo, hi = stl_bounds(filename)
        lo, hi = lo * scale, hi * scale
    else:
        from Hannah_Work.wall_collision import dae_object_boxes
        parts = dae_object_boxes(filename, np.diag([*scale, 1.]))
        lo = np.min([part[1] for part in parts], axis=0)
        hi = np.max([part[2] for part in parts], axis=0)
    return np.maximum(hi-lo, .002), SE3(*((lo+hi)/2))


class WorkcellRecovery(MaryamRecovery):
    """All four arms: pure preflight, fixed mobile bases, explicit payloads."""
    def __init__(self, runtime, movements):
        super().__init__(runtime, movements)
        self.home_q = {}
        self.saved = {}
        self.static_boxes = []
        self.last_blocked = ''
        self._payload_normals = {}

    # Save startup joints for all loaded arms and collect selected scene-solid collision boxes.
    def register_all(self):
        for name, robot in self.runtime.robots.items():
            self.home_q[name] = np.asarray(robot.q, float).copy()
        from Hannah_Work.wall_collision import dae_object_boxes
        names = ('botCounter', 'ConvCounter', 'ExtraCounter', 'HannahCounter',
                 'backWall', 'backWall2', 'LeftWall', 'rightWall', 'backWall_meal')
        for name in names:
            mesh = self.runtime.objects.get(name)
            if mesh is None:
                continue
            transform = np.asarray(mesh.T) @ np.diag([*np.resize(mesh.scale, 3), 1.])
            for part, lo, hi in dae_object_boxes(mesh.filename, transform):
                self.static_boxes.append((f'{name}:{part}', SE3(*((lo+hi)/2)), hi-lo))

    def tool(self, name, q):
        return self.runtime.bridge.tool_pose(name, q)

    # Predict link/gripper volumes using each model's own kinematics without moving it.
    def body_boxes(self, name, q):
        robot = self.runtime.robots[name]
        if name == 'MaryamBot':
            return self.delivery.controller.body_poses(q)
        if name == 'HannahBot':
            transforms = robot.get_mesh_joint_transforms(q)
            meshes = list(robot.link_meshes) + [robot.gripper_mesh]
            return [(robot.base * transforms[min(i, 5)] * centre, size)
                    for i, mesh in enumerate(meshes)
                    for size, centre in [mesh_box(mesh.filename, tuple(np.resize(mesh.scale, 3)))]]
        if name == 'ReBot':
            links = robot.get_link_transforms(q)
            grippers = robot.get_gripper_transforms(links, robot.gripper_opening)
            motions = [links[i] * robot.home_transforms[i].inv() for i in range(6)]
            motions += [grippers[i] * robot.home_gripper_transforms[i].inv() for i in range(3)]
            return [(robot.base * motion * robot.original_mesh_poses[i+1] * centre, size)
                    for i, (mesh, motion) in enumerate(zip(robot.meshes[1:], motions))
                    for size, centre in [mesh_box(mesh.filename, tuple(np.resize(mesh.scale, 3)))]]
        # Yara: conservative link-segment boxes, matching the existing centreline checks.
        points = [robot.base.t.copy()]
        frame = SE3(robot.base)
        for link, angle in zip(robot.links, q):
            frame = frame * link.A(float(angle))
            points.append(frame.t.copy())
        result = []
        for a, b in zip(points, points[1:]):
            delta = b-a
            length = np.linalg.norm(delta)
            z = delta / length if length > 1e-9 else np.array([0., 0., 1.])
            helper = np.array([1., 0., 0.]) if abs(z[0]) < .9 else np.array([0., 1., 0.])
            x = np.cross(helper, z); x /= np.linalg.norm(x)
            rotation = np.column_stack((x, np.cross(z, x), z))
            result.append((SE3.Rt(rotation, (a+b)/2), np.array([.06, .06, length+.06])))
        return result

    # SHOW_ATTACHMENTS - Return explicitly held objects and their tool-relative transforms; do not guess from proximity.
    def payloads(self, name):
        if name == 'MaryamBot':
            d = self.delivery
            if d._carried is None:
                return []
            return [(d._carried, d._relative)] + [(shape, d._relative * relative)
                                                 for shape, relative in d.payload]
        return list(getattr(self.runtime.robots[name], 'recovery_payloads', []))

    # Move registered held objects with the achieved world tool pose during recovery/manual motion.
    def update_payloads(self, name):
        tool = self.tool(name, self.runtime.robots[name].q)
        for shape, relative in self.payloads(name):
            shape.T = (tool * relative).A

    # SHOW_COLLISION - Check limits, model guards, held-item tilt and selected scene/nearby-arm volumes.
    def clear(self, name, q):
        robot = self.runtime.robots[name]
        lo, hi = self.runtime.bridge.limits_deg(name)[0]
        if not np.all(np.isfinite(q)) or np.any(q < np.deg2rad(lo)-1e-9) or np.any(q > np.deg2rad(hi)+1e-9):
            self.last_blocked = f'{name}: joint limits'
            return False
        if name == 'MaryamBot' and not self.delivery.recovery_clear(q):
            self.last_blocked = 'MaryamBot: arm/tray/queued-tray constraint'
            return False
        if name == 'YaraBot':
            from Collective_Folder import yara_main_testing as yara
            held = self.payloads(name)
            if not yara.configuration_safe(q, *(held[0] if held else (None, None)))[0]:
                self.last_blocked = 'YaraBot: existing collision guard'
                return False
        bodies = self.body_boxes(name, q)
        payload_bodies = []
        for shape, relative in self.payloads(name):
            pose = self.tool(name, q) * relative
            normal = self._payload_normals.get(id(shape), np.asarray(shape.T)[:3,:3].T @ [0.,0.,1.])
            if (pose.R @ normal)[2] < np.cos(np.deg2rad(10)):
                self.last_blocked = f'{name}: held item would tilt'
                return False
            if getattr(shape, 'filename', None):
                size, centre = mesh_box(shape.filename, tuple(np.resize(shape.scale, 3)))
            else:
                size, centre = np.asarray(shape.scale), SE3()
            payload_bodies.append((pose * centre, size))
        obstacles = list(self.static_boxes)
        for other, model in self.runtime.robots.items():
            if other != name and np.linalg.norm(robot.base.t-model.base.t) < 2.2:
                obstacles += [(other, pose, size) for pose, size in self.body_boxes(other, model.q)]
        for pose, size in bodies + payload_bodies:
            for label, obstacle, extent in obstacles:
                if boxes_overlap(pose, size, obstacle, extent, margin=.003):
                    self.last_blocked = f'{name}: {label}'
                    return False
        # Contact at the grasping assembly is expected. Check a held item's
        # volume against the rest of its own arm, not the fingers gripping it.
        free_links = {'ReBot': 6, 'HannahBot': 5, 'YaraBot': 4}.get(name, 0)
        for pose, size in payload_bodies:
            for link, extent in bodies[:free_links]:
                if boxes_overlap(pose, size, link, extent, margin=.002):
                    self.last_blocked = f'{name}: held item intersects its own arm'
                    return False
        return self.runtime.configuration_clear(name, q)

    # SHOW_HOME - Select one or all saved startup arm configurations and request checked recovery.
    def home_robot(self, name='All'):
        if getattr(self.runtime, 'scenario', None) and self.runtime.scenario.active:
            raise ValueError('Finish the collision scenario before Home')
        names = list(self.runtime.robots) if name == 'All' else [name]
        targets = {n: self.home_q[n] for n in names}
        self._recover(targets, remember=True)

    # SHOW_RECOVERY - Plan all requested routes before moving; recheck each execution sample and stop on changed hazards.
    def _recover(self, targets, remember=False):
        rt = self.runtime
        if rt.safety and rt.safety.check():
            raise ValueError('Clear the safety intrusion before recovery')
        previous, generation = rt.state, rt.stop_generation
        self._payload_normals = {id(shape): np.asarray(shape.T)[:3,:3].T @ [0.,0.,1.]
            for name in targets for shape,_ in self.payloads(name)}
        rt.set_state('RECOVERING', 'Prechecking all requested arms; mobile bases remain stationary')
        def pulse():
            if rt.closed or rt.state != 'RECOVERING' or rt.stop_generation != generation:
                raise ValueError('Recovery interrupted by stop')
        try:
            routes = {}
            for name, goal in targets.items():
                pulse()
                start = rt.robots[name].q.copy()
                if np.allclose(start, goal, atol=1e-9):
                    routes[name] = [start]
                    continue
                route = plan_joint_route(start, goal, lambda q,n=name: self.clear(n,q),
                    *np.deg2rad(rt.bridge.limits_deg(name)[0]), pulse=pulse)
                if route is None:
                    raise ValueError(f'Home rejected before movement: no verified {name} route ({self.last_blocked})')
                routes[name] = route
            if remember:
                for name in targets:
                    saved = rt.bridge.saved.get(name)
                    if (len(routes[name]) == 1 and name not in self.saved
                            and (saved is None or np.allclose(saved[0], rt.robots[name].q))):
                        continue  # Already home; do not lock out Manual with an unnecessary return pose.
                    if name not in self.saved:
                        self.saved[name] = ((saved[0] if saved else rt.robots[name].q).copy(),
                                            SE3(rt.robots[name].base))
                    rt.bridge.saved.pop(name, None)
            for name, route in routes.items():
                rt.log(f'{name}: checked recovery')
                for q in route[1:]:
                    pulse()
                    if not self.clear(name, q):
                        rt.request_stop(self.last_blocked)
                        raise ValueError('Recovery path became blocked')
                    robot = rt.robots[name]
                    robot.q = np.asarray(q).copy()
                    if hasattr(robot, 'update_meshes'):
                        robot.update_meshes()
                    self.update_payloads(name)
                    rt.motion_step(.025)
        finally:
            self._payload_normals = {}
            if rt.state == 'RECOVERING':
                rt.set_state(previous, 'Recovery ended. Press Start / Resume separately.')

    # SHOW_RESUME_POSE - Restore interrupted task joints at unchanged bases before runtime allows automatic continuation.
    def restore(self):
        for name, (_, base) in self.saved.items():
            if not np.allclose(self.runtime.robots[name].base.A, base.A):
                raise ValueError(f'{name} base changed during recovery')
        if self.saved:
            self._recover({name: state[0] for name, state in reversed(list(self.saved.items()))})
            self.saved.clear()
