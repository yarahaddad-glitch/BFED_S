# Two demos: a pending-path sphere test and a separate paused tray/cube scenario.
# Detection identifies rejected configurations; avoidance searches a checked replacement. Obstacles stay present during detour execution.

import numpy as np
import time
from spatialgeometry import Sphere, Cuboid, Mesh
from spatialmath import SE3
from Maryam_Work.joint_planner import plan_joint_route, samples


class MaryamCollisionDemo:
    # Connect before-path, replan and blocked-wait handlers to the real TrayDelivery object.
    def __init__(self, runtime, movements):
        self.runtime = runtime
        self.delivery = movements.delivery
        self.controller = movements.controller
        self.robot = self.delivery.robot
        self.armed = False
        self.blocked = False
        self.obstacle = None
        self.marker = None
        self.status = 'Ready'
        self.avoid = False
        self.last_route = None
        self.route_markers = []
        self.delivery.before_path = self.before_path
        self.delivery.wait_for_clearance = self.wait_for_clearance
        self.delivery.replan_path = self.replan_path

    def arm_test(self, avoid=False):
        if self.runtime.state not in ('READY', 'PAUSED', 'COMPLETE'):
            raise ValueError('Pause before inserting an obstacle')
        if self.obstacle is not None:
            raise ValueError('Remove the previous obstacle first')
        self.armed = True
        self.avoid = avoid
        self.status = 'Armed: start a meal'
        self.runtime.log('Collision test armed')
        path = self.delivery.remaining_path
        if path is not None:
            self.before_path(path)

    # Insert a sphere into a future checked link volume while keeping the present configuration clear.
    def before_path(self, path):
        if not self.armed or len(path) < 3:
            return
        # Place the object inside a future link volume, checking that it is
        # outside the current arm and tray. This is a real geometry test,
        # not an unconditional simulated stop.
        indices = range(1, len(path) - 1, max(1, len(path) // 16))
        for index in sorted(indices, key=lambda i: abs(i - (len(path) - 1) / 2)):
            q = path[index]
            for body, _ in reversed(self.controller.body_poses(q)):
                candidate = dict(type='sphere', center=body.t.copy(), radius=0.055)
                self.controller.obstacles.append(candidate)
                current_clear = self.delivery.configuration_clear(self.robot.q)
                future_blocked = not self.delivery.configuration_clear(q)
                goal_clear = self.delivery.configuration_clear(path[-1])
                if current_clear and future_blocked and (goal_clear or not self.avoid):
                    self.obstacle = candidate
                    self.armed = False
                    self.blocked = not self.avoid
                    if self.marker is None:
                        self.marker = Sphere(radius=candidate['radius'],
                            pose=SE3(*candidate['center']), color=[1., .15, .1, 1.])
                        self.runtime.env.add(self.marker)
                    else:
                        self.marker.T = SE3(*candidate['center']).A
                    self.runtime.bridge.draw(0.025)
                    self.status = ('Obstacle ahead: planning detour' if self.avoid
                                   else 'Obstacle in path: stopped')
                    self.runtime.log('MaryamBot: obstacle ahead')
                    if not self.avoid:
                        self.runtime.request_stop('MaryamBot planned path blocked by test obstacle')
                    return
                self.controller.obstacles.pop()
        self.status = 'Armed: waiting for a leg with a clear start and destination'

    # Use joint_planner with payload-aware validity to replace a blocked pending leg.
    def replan_path(self, path):
        if not self.avoid or self.obstacle is None or self.blocked:
            return None
        self.status = 'Planning checked detour; robot stationary'
        self.runtime.log('MaryamBot: replanning around retained obstacle')
        route = plan_joint_route(self.robot.q.copy(), np.asarray(path[-1]),
            self.delivery.recovery_clear, self.controller.q_min, self.controller.q_max,
            pulse=self.runtime.checkpoint)
        if route is None:
            self.status = 'No verified detour: stopped before contact'
            return None
        self.last_route = np.asarray(route).copy()
        for marker in self.route_markers:
            marker.T = SE3(0, 0, -100).A
        self.route_markers = []
        for q in route[::max(1, len(route) // 18)]:
            point = self.robot.fkine(q).t
            marker = Sphere(radius=.012, pose=SE3(*point), color=[.1, .85, .35, .9])
            self.runtime.env.add(marker)
            self.route_markers.append(marker)
        self.status = 'Avoiding obstacle on verified route'
        self.runtime.log('MaryamBot: verified detour found; obstacle remains in scene')
        return route

    def active_hazards(self):
        return ['MaryamBot path blocked'] if self.blocked and self.obstacle is not None else []

    # Latch stop on blocked movement and wait through runtime checkpoints without dropping task state.
    def wait_for_clearance(self):
        self.blocked = True
        self.status = 'Path blocked: remove obstacle'
        if self.runtime.state != 'ESTOP':
            self.runtime.log('MaryamBot: path blocked')
            self.runtime.request_stop('MaryamBot: tray or arm path blocked')
        # Keeps the original trajectory and payload state on the call stack.
        # Condition waiting services Remove / Reset / Start without a busy loop.
        self.runtime.checkpoint()

    def remove_obstacle(self):
        if self.runtime.state == 'RUNNING':
            raise ValueError('Pause before removing an obstacle')
        self.controller.obstacles[:] = [obs for obs in self.controller.obstacles
                                        if obs is not self.obstacle]
        self.obstacle = None
        self.armed = self.blocked = False
        if self.marker is not None:
            self.marker.T = SE3(0, 0, -100).A
        for marker in self.route_markers:
            marker.T = SE3(0, 0, -100).A
        self.route_markers = []
        self.runtime.bridge.draw(0.025)
        self.status = 'Obstacle removed'
        self.runtime.log('Obstacle removed')


class TrayCollisionScenario:
    """Operator-controlled tray task on the real arm, with an editable cube.

    Other jobs remain parked. A demonstration tray borrows a stack location;
    inventory and the interrupted automatic call stack are restored on Finish.
    """
    def __init__(self, runtime, movements):
        self.runtime, self.movements = runtime, movements
        self.robot = movements.delivery.robot
        self.active = False
        self.status = 'Choose Deposit or Handout, then prepare the scenario.'
        self.detection = 'No route checked yet'
        self.markers = []
        self.cube = None
        self.centre = np.zeros(3)
        self.side = .10
        self.revision = 0
        self.route = None
        self.finished = self.interrupted = self.has_moved = False
        self.rng = np.random.default_rng(41)

    def pulse(self):
        rt = self.runtime
        if rt.closed or rt.state == 'ESTOP':
            raise ValueError('Scenario stopped. Reset, then Finish / Restore before another attempt.')
        if rt.safety and rt.safety.check():
            raise ValueError('Clear the safety intrusion before continuing the scenario')

    def _hide_markers(self):
        for mesh in self.markers:
            mesh.T = SE3(0, 0, -100).A
        self.markers = []

    # SHOW_PREPARE - Preserve the paused real task and build a separate demonstration tray/reference transfer.
    def prepare(self, task='deposit'):
        from Maryam_Work.maryam_movements import (build_controller, mesh_xyz,
            HANDOUT_GAP_LOCAL, HANDOUT_SIDE_CLEARANCE, HANDOUT_INSERTION,
            BELT_SURFACE_Z, serving_gap_validator)
        from Maryam_Work.maryam_tray_delivery import TrayDelivery
        from types import SimpleNamespace
        rt = self.runtime
        if rt.state not in ('READY', 'PAUSED', 'COMPLETE'):
            raise ValueError('Pause and reset any stop before preparing a scenario')
        if self.active:
            raise ValueError('Finish / Restore the current scenario first')
        if self.movements.delivery._carried is not None:
            raise ValueError('Maryam is carrying a real meal. Resume and finish it before this separate scenario.')
        if rt.collision.obstacle is not None:
            raise ValueError('Remove the previous automatic collision-test obstacle first')
        pick = self.movements.stacks.next_tray('blue')
        if pick is None:
            raise ValueError('No blue demonstration source remains; rebuild the simulation first')
        self.saved_q = self.robot.q.copy()
        self.source = pick[1]
        self.source_pose = SE3(self.source.T)
        self.saved_armed = rt.collision.armed
        rt.collision.armed = False
        self.active = True
        self.finished = self.interrupted = self.has_moved = False
        self.route = None
        self.task = task
        self.status = 'Preparing a collision-free reference task; arm stationary'
        source_pose = self.source_pose
        self.controller = build_controller(self.robot)
        self.controller.checkpoint = self.pulse
        self.delivery = TrayDelivery(SimpleNamespace(step=rt.motion_step), self.robot,
            self.controller, self.movements.layout, tray_scale=2,
            animation_speed=self.movements.delivery.animation_speed,
            cache_dir=self.movements.delivery.cache_dir)
        if task == 'handout':
            source_pose = SE3.Rt(self.source_pose.R,
                self.movements.layout.place_centre-self.source_pose.R@self.delivery.centre_local)
        self.source.T = SE3(0, 0, -100).A
        self.tray = Mesh(self.source.filename, scale=list(self.source.scale), pose=source_pose)
        rt.env.add(self.tray)
        self.food = []
        if task == 'handout':
            for dx, colour in ((-.09, [.3,.7,.2,1]), (.09, [.9,.7,.2,1])):
                relative = SE3(*(self.delivery.centre_local + [dx, 0, .048]))
                shape = Cuboid(scale=[.12,.12,.025], pose=source_pose*relative, color=colour)
                rt.env.add(shape)
                self.food.append((shape, relative))
        self.delivery.payload = list(self.food)
        self.delivery.stationary_trays = [tray for tray in self.movements.stacks.trays.values()
                                         if tray is not self.source]
        self.controller.extra_valid = lambda q: self.delivery.queued_trays_clear(q,
            active_tray=self.tray) and rt.recovery.clear('MaryamBot', q)
        self.old_validator = rt.bridge.validators.get('MaryamBot')
        self.old_update = rt.bridge.motion_updates.get('MaryamBot')
        self.arguments = {}
        if task == 'handout':
            gap = mesh_xyz(self.movements.objects, 'frontBars', (13.4,4.7,.5)) + HANDOUT_GAP_LOCAL
            inside = gap + [.06, -HANDOUT_SIDE_CLEARANCE, 0]
            align = inside + [0, -.15, 0]
            align[2] = BELT_SURFACE_Z + .18
            low = align.copy(); low[2] = inside[2]
            self.arguments = dict(destination=gap+[.06,HANDOUT_INSERTION,0],
                waypoints=[align,low,inside], carry_validator=serving_gap_validator(gap),
                destination_rotation=SE3.Rz(-np.pi/2).R@source_pose.R)
        try:
            prepared = self.delivery.prepare(self.tray, **self.arguments)
            if prepared[-1] is None:
                raise ValueError('No verified reference task; try Deposit or restore a clear starting pose')
            approach, descent, legs, self.relative, hold = prepared[-1]
            self.delivery._relative = self.relative
            self.delivery._carry_validator = self.arguments.get('carry_validator')
            self.start_pose = SE3(self.tray.T)
            self.end_pose = self.robot.fkine(legs[-1][1][-1]) * self.relative
            self.phases = [('approach', approach, False, self.start_pose),
                           ('descend to grip', descent, False, self.start_pose)]
            self.phases += [(name,path,True,self.start_pose) for name,path in legs]
            retreats = [legs[-1]] if task == 'deposit' else [legs[-1],legs[-2]]
            self.phases += [('retreat',list(reversed(path)),False,self.end_pose) for _,path in retreats]
            self.shadow = SimpleNamespace(T=self.start_pose.A.copy())
            self.place_random_cube()
            self._configure(self.phases[0])
            rt.bridge.validators['MaryamBot'] = self.delivery.configuration_clear
            rt.bridge.motion_updates['MaryamBot'] = self.delivery.update_payload
        except Exception:
            # No robot movement occurred during preparation.
            self._hide_scene()
            self.source.T = self.source_pose.A
            self.active = False
            rt.collision.armed = self.saved_armed
            raise

    def _configure(self, phase, actual=False):
        _, _, attached, static_pose = phase
        item = self.tray if actual else self.shadow
        if not actual:
            item.T = static_pose.A.copy()
        self.delivery._watched_tray = item
        self.delivery._carried = item if attached else None

    # Use the scenario delivery's payload-aware recovery constraints for proposed joints.
    def valid(self, q):
        return self.delivery.recovery_clear(q)

    def place_random_cube(self):
        if not self.active or self.has_moved:
            raise ValueError('Prepare a fresh scenario before random placement')
        self.controller.obstacles = []
        # Prefer translational carrying legs. The constrained wrist-turn arc at
        # the serving aperture may have no feasible detour around a cube.
        roomy = ('lift', 'move to turning clearance', 'over belt', 'clear robot beside belt')
        phases = [phase for phase in self.phases if phase[2] and phase[0] in roomy]
        self.rng.shuffle(phases)
        phases += [phase for phase in self.phases if phase[2] and phase[0] not in roomy]
        if self.task == 'handout':
            # Default to the approach to the loaded tray: a useful handout
            # obstruction with space for an alternate arm route. The operator
            # can still move the cube into the narrow serving aperture.
            phases = [phase for phase in self.phases if phase[0] == 'approach'] + phases
        for phase in phases:
            path = phase[1]
            indices = list(range(max(1,len(path)//5), max(2,len(path)*4//5), max(1,len(path)//9)))
            self.rng.shuffle(indices)
            for index in indices:
                q = path[min(index,len(path)-1)]
                tray_pose = self.robot.fkine(q)*self.relative
                candidates = [tray_pose.t+tray_pose.R@self.delivery.centre_local+[0,0,.04]]
                for pose,size in reversed(self.controller.body_poses(q)):
                    candidates.append(pose.t)
                    for axis in range(3):
                        for sign in (-1,1):
                            candidates.append(pose.t + sign*.4*size[axis]*pose.R[:,axis])
                for centre in candidates:
                    obstacle = dict(type='box', pose=SE3(*centre), size=[self.side]*3)
                    self.controller.obstacles = [obstacle]
                    self._configure(phase)
                    blocked = not self.valid(q)
                    endpoints_clear = True
                    for other in self.phases:
                        self._configure(other)
                        if not self.valid(other[1][-1]):
                            endpoints_clear = False
                            break
                    self._configure(self.phases[0])
                    if blocked and endpoints_clear and self.valid(self.robot.q):
                        self.set_cube(centre, self.side)
                        self.detection = f'Cube is in the predicted {phase[0]} sweep; press Check + Plan.'
                        return
                self.pulse()
        self.controller.obstacles = []
        self.set_cube(self.robot.fkine(self.robot.q).t+[.25,0,.12], self.side)
        self.detection = 'No suitable random blocked sweep found. Edit cube XYZ, then Check + Plan.'

    # SHOW_OBSTACLE - Update visible cube/collision geometry together; invalidate the previous route revision.
    def set_cube(self, centre, side):
        if not self.active or self.has_moved:
            raise ValueError('Cube editing is available before execution; Finish / Restore to start again')
        centre = np.asarray(centre,float)
        if centre.shape != (3,) or not np.all(np.isfinite(centre)) or not .04 <= side <= .20:
            raise ValueError('Use finite world XYZ and a cube side between 0.04 and 0.20 m')
        if self.cube is not None:
            self.cube.T = SE3(0,0,-100).A
        self.centre, self.side = centre.copy(), float(side)
        self.obstacle = dict(type='box', pose=SE3(*centre), size=[side]*3)
        self.controller.obstacles = [self.obstacle]
        self.cube = Cuboid(scale=[side]*3, pose=SE3(*centre), color=[1,.32,.08,1])
        self.runtime.env.add(self.cube)
        self.revision += 1
        self.route = None
        self._hide_markers()
        self.status = 'Cube placed. Arm paused; Check + Plan before Execute.'
        self.runtime.motion_step(.025)

    # Check carried-tray attitude, arm/waiting-tray/obstacle clearance and serving aperture.
    def _payload_valid(self, q):
        """Payload-only predicate for RMRC; never calls controller.valid recursively."""
        d = self.delivery
        if d._carried is None:
            return True
        pose = self.robot.fkine(q)*self.relative
        aperture = self.arguments.get('carry_validator')
        return (pose.R[2,2] >= np.cos(np.deg2rad(8))
            and d.tray_clears_robot(q,pose)
            and d.queued_trays_clear(q,pose,self.tray)
            and d.tray_clears_obstacles(pose)
            and (aperture is None or aperture(pose)))

    # Try constrained Cartesian RMRC detours while preserving payload requirements, then check the goal connection.
    def _cartesian_detour(self, start, goal):
        # A level loaded tray occupies a constrained subset of joint space.
        # RMRC can keep it level through the 90-degree handout turn, where
        # arbitrary joint interpolation is usually unsuitable.
        old = self.controller.extra_valid
        old_checkpoint = self.controller.checkpoint
        deadline = time.monotonic() + 8.
        def bounded_pulse():
            self.pulse()
            if time.monotonic() > deadline:
                raise TimeoutError('Cartesian detour search budget exhausted')
        self.controller.extra_valid = lambda q: old(q) and self._payload_valid(q)
        self.controller.checkpoint = bounded_pulse
        try:
            target = self.robot.fkine(goal)
            route, method = self.controller.plan(target.t,target.R,q_start=start)
            if route is None:
                return None
            tail = samples(np.asarray(route[-1]),goal)
            if not all(self.valid(q) for q in tail):
                return None
            return np.vstack((route,tail[1:]))
        except TimeoutError:
            return None
        finally:
            self.controller.extra_valid = old
            self.controller.checkpoint = old_checkpoint

    # Explain whether rejection came from a link, carried tray or another task constraint.
    def _collision_detail(self, q):
        if self.controller.colliding(q):
            return self.controller.last_collision
        d = self.delivery
        if d._carried is not None:
            pose = self.robot.fkine(q)*self.relative
            if not d.tray_clears_obstacles(pose):
                return 'carried tray / meal envelope intersects cube'
            if not d.tray_clears_robot(q,pose):
                return 'carried tray intersects arm or fixed housing'
        return 'queued-tray, serving-aperture or scene-clearance constraint'

    # SHOW_DETECTION - Densely check reference phases; for a blocked phase try Cartesian then joint-space detours.
    def plan(self):
        if not self.active or self.has_moved:
            raise ValueError('Prepare a new scenario before planning')
        self.pulse()
        self.route = None
        self._hide_markers()
        self.status = 'Checking the full original task, including arm and attached tray'
        self.detection = 'No collision found on the checked reference task'
        original_q = self.robot.q.copy()
        routes = []
        start = original_q.copy()
        found = []
        for phase in self.phases:
            self.pulse()
            self._configure(phase)
            original = np.asarray(phase[1])
            knots = np.vstack((start, original))
            dense = [knots[0]]
            for a,b in zip(knots,knots[1:]):
                dense.extend(samples(a,b)[1:])
            dense = np.asarray(dense)
            hit = next((i for i,q in enumerate(dense) if not self.valid(q)), None)
            if hit is not None:
                found.append(phase[0])
                detail = self._collision_detail(dense[hit])
                self.detection = ('DETECTED: '+phase[0]+f', sample {hit}: '+detail)
                self.status = f'Planning around retained cube: {phase[0]}'
                self.runtime.log(self.detection)
                route = self._cartesian_detour(start,original[-1]) if phase[2] else None
                if route is None:
                    route = plan_joint_route(start, original[-1], self.valid,
                        self.controller.q_min, self.controller.q_max, pulse=self.pulse)
                if route is None:
                    self.status = f'BLOCKED: no verified {phase[0]} detour. Arm did not move. Edit cube XYZ or Finish.'
                    self._configure(self.phases[0])
                    return False
            else:
                route = dense
            routes.append((phase,np.asarray(route)))
            start = np.asarray(route[-1]).copy()
        self.route = routes
        self.plan_revision = self.revision
        self.plan_start_q = original_q
        self._configure(self.phases[0])
        self.status = ('VERIFIED DETOUR: cube retained. Press Execute.' if found else
                       'Original task is clear. Move the cube into its sweep to demonstrate avoidance.')
        points = np.vstack([path for _,path in routes])
        for q in points[::max(1,len(points)//35)]:
            marker = Sphere(radius=.009, pose=self.robot.fkine(q), color=[.12,.8,.3,1])
            self.runtime.env.add(marker)
            self.markers.append(marker)
        self.runtime.motion_step(.025)
        return True

    # SHOW_EXECUTION - Reject stale/changed plans; precheck then execute samples with the cube retained and other jobs parked.
    def execute(self):
        if self.interrupted or self.finished:
            raise ValueError('Finish / Restore this attempt before another execution')
        if self.route is None or self.plan_revision != self.revision:
            raise ValueError('Check + Plan the current cube position first')
        if not np.allclose(self.robot.q,self.plan_start_q,atol=1e-8):
            self.route = None
            raise ValueError('Arm was jogged after planning. Press Check + Plan again.')
        rt = self.runtime
        previous = rt.state
        if previous not in ('READY','PAUSED','COMPLETE'):
            raise ValueError('Reset the stop before executing the scenario')
        for phase,path in self.route:
            self._configure(phase)
            if not all(self.valid(q) for q in path):
                self.route = None
                raise ValueError('Scene changed after planning. Nothing moved; check again.')
        rt.set_state('DEMONSTRATION','Maryam tray scenario; other robot jobs remain parked')
        try:
            for phase,path in self.route:
                self._configure(phase,actual=True)
                self.status = 'Executing: '+phase[0]
                for q in path[1:]:
                    self.pulse()
                    if not self.valid(q):
                        rt.request_stop('Scenario arm/tray path became blocked')
                        raise ValueError('Scenario stopped before assigning a blocked configuration')
                    self.robot.q = q.copy()
                    self.has_moved = True
                    self.delivery.update_payload()
                    rt.motion_step(self.controller.dt/self.delivery.animation_speed)
            self.delivery._carried = None
            self.finished = True
            self.status = 'Task completed with cube retained. Finish / Restore returns to the automatic task.'
        except Exception:
            self.interrupted = True
            self.status = 'Attempt interrupted. Reset the stop if active, then Finish / Restore.'
            raise
        finally:
            if rt.state == 'DEMONSTRATION':
                rt.set_state(previous, 'Collision scenario ended; automation remains paused')

    def _hide_scene(self):
        self._hide_markers()
        for mesh in [getattr(self,'cube',None),getattr(self,'tray',None)] + [x[0] for x in getattr(self,'food',[])]:
            if mesh is not None:
                mesh.T = SE3(0,0,-100).A

    # SHOW_RESTORE - Remove only demonstration props and check the return to the saved real-task configuration.
    def finish(self):
        if not self.active:
            return
        rt = self.runtime
        if rt.state not in ('READY','PAUSED','COMPLETE'):
            raise ValueError('Reset the stop before Finish / Restore')
        self.controller.obstacles = []
        self.delivery._carried = self.delivery._watched_tray = None
        self._hide_scene()  # Explicitly remove the demonstration props; real inventory is still parked.
        route = plan_joint_route(self.robot.q.copy(),self.saved_q,self.controller.valid,
            self.controller.q_min,self.controller.q_max,pulse=self.pulse)
        if route is None:
            raise ValueError('No verified restore route. Scenario remains paused; jog to a clear pose and retry Finish.')
        previous = rt.state
        rt.set_state('RECOVERING','Restoring Maryam after the isolated tray scenario')
        try:
            for q in route[1:]:
                self.pulse()
                if not self.controller.valid(q):
                    rt.request_stop('Scenario restore path became blocked')
                    raise ValueError('Restore blocked')
                self.robot.q = q.copy()
                rt.motion_step(.025)
            self.source.T = self.source_pose.A
            rt.bridge.validators['MaryamBot'] = self.old_validator
            rt.bridge.motion_updates['MaryamBot'] = self.old_update
            rt.collision.armed = self.saved_armed
            self.active = False
            self.status = 'Scenario closed. Real tray inventory and paused task restored.'
            self.route = None
        finally:
            if rt.state == 'RECOVERING':
                rt.set_state(previous, 'Scenario restored. Automatic Start / Resume remains a separate action.')
        rt.motion_step(.025)
