"""Predictive obstacle test on Maryam's actual pending tray trajectory."""
import numpy as np
from spatialgeometry import Sphere
from spatialmath import SE3


class MaryamCollisionDemo:
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
        self.delivery.before_path = self.before_path
        self.delivery.wait_for_clearance = self.wait_for_clearance

    def arm_test(self):
        if self.runtime.state not in ('READY', 'PAUSED', 'COMPLETE'):
            raise ValueError('Pause before inserting an obstacle')
        if self.obstacle is not None:
            raise ValueError('Remove the previous obstacle first')
        self.armed = True
        self.status = 'Armed: start a meal'
        self.runtime.log('Collision test armed')
        path = self.delivery.remaining_path
        if path is not None:
            self.before_path(path)

    def before_path(self, path):
        if not self.armed or len(path) < 3:
            return
        # Place the object inside a future link volume, checking that it is
        # outside the current arm and tray. This is a real geometry test,
        # not an unconditional simulated stop.
        for q in list(path)[::-max(1, len(path) // 12)]:
            for body, _ in reversed(self.controller.body_poses(q)):
                candidate = dict(type='sphere', center=body.t.copy(), radius=0.055)
                self.controller.obstacles.append(candidate)
                current_clear = self.delivery.configuration_clear(self.robot.q)
                future_blocked = not self.delivery.configuration_clear(q)
                if current_clear and future_blocked:
                    self.obstacle = candidate
                    self.armed = False
                    self.blocked = True
                    if self.marker is None:
                        self.marker = Sphere(radius=candidate['radius'],
                            pose=SE3(*candidate['center']), color=[1., .15, .1, 1.])
                        self.runtime.env.add(self.marker)
                    else:
                        self.marker.T = SE3(*candidate['center']).A
                    self.runtime.bridge.draw(0.025)
                    self.status = 'Obstacle in path: stopped'
                    self.runtime.log('MaryamBot: obstacle ahead')
                    self.runtime.request_stop('MaryamBot planned path blocked by test obstacle')
                    return
                self.controller.obstacles.pop()

    def active_hazards(self):
        return ['MaryamBot path blocked'] if self.blocked and self.obstacle is not None else []

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
            self.runtime.bridge.draw(0.025)
        self.status = 'Obstacle removed'
        self.runtime.log('Obstacle removed')
