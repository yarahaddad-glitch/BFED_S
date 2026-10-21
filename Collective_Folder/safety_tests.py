"""Shared-scene intrusion demonstrations, separate from collision tests.

Imported by main_all without launching a scene or moving any actor.
Run this file directly to open the integrated GUI, then choose Safety tests.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
from Collective_Folder.Light_curtain import SafetyLightCurtain
from Collective_Folder.Prison_break import PrisonBreak

KITCHEN_SPOT = (13.0, 2.0, 0.0)
ISOLATION_SPOT = (6.0, 15.0, 0.0)


class SafetyDemonstration:
    def __init__(self, env, scene, runtime):
        self.env, self.scene, self.runtime = env, scene, runtime
        self.actor = scene['prisoner1']
        self.saved_pose = None
        self.start_pose = np.array(self.actor.T, float).copy()
        self.curtain = SafetyLightCurtain(env, on_trip=self.trip)
        self.curtain.watch_prisoners([scene[name] for name in
            ('prisoner1', 'prisoner2', 'prisoner3') if name in scene])
        self.zones = PrisonBreak(env, scene, on_trip=self.trip)
        self.status = 'CLEAR'
        self.last_event = 'Ready: light curtain and unsafe-zone monitoring active.'
        self.alert_number = 0

    def trip(self, reason):
        self.last_event = reason
        self.alert_number += 1
        self.runtime.log('Prisoner in area')
        self.runtime.request_stop(reason)

    def active_hazards(self):
        beams = [f'Light curtain: {name}' for name in sorted(self.curtain.intrusions())]
        zones = [f'{name} in {zone}' for name, zone in sorted(self.zones.intrusions())]
        return beams + zones

    def check(self):
        self.curtain.check()
        self.zones.check()
        hazards = self.active_hazards()
        self.status = 'TRIPPED: ' + '; '.join(hazards) if hazards else 'CLEAR'
        return hazards

    def arm(self):
        step = self.env.step

        def guarded_step(*args, **kwargs):
            result = step(*args, **kwargs)
            self.check()
            return result

        self.env.step = guarded_step

    def run(self, kind):
        if self.runtime.state not in ('READY', 'PAUSED', 'COMPLETE'):
            raise ValueError('Clear the test actor and acknowledge the stop before another test')
        if self.saved_pose is not None:
            raise ValueError('Clear the previous test actor first')
        if kind not in ('curtain', 'kitchen', 'isolation'):
            raise ValueError('Unknown safety demonstration')
        self.saved_pose = np.array(self.actor.T, float).copy()
        self.last_event = f'Running {kind} intrusion test...'
        if kind == 'curtain':
            # Same four-metre approach as the original safety_tests.py. End the
            # animation immediately at detection, leaving the intruder visible.
            start = self.start_pose.copy()
            for distance in np.linspace(0, 4.0, 81):
                if self.runtime.closed or self.runtime.state == 'ESTOP':
                    break
                pose = start.copy()
                pose[1, 3] -= distance
                self.actor.T = pose
                self.runtime.bridge.draw(0.025)
        else:
            pose = np.array(self.actor.T, float).copy()
            pose[:3, 3] = KITCHEN_SPOT if kind == 'kitchen' else ISOLATION_SPOT
            self.actor.T = pose
            self.runtime.bridge.draw(0.025)
        if self.runtime.state != 'ESTOP':
            raise ValueError('Safety test did not trigger; inspect the configured detection geometry')

    def clear_actor(self):
        if self.runtime.state == 'RUNNING':
            raise ValueError('Pause before clearing the safety test actor')
        if self.saved_pose is not None:
            self.actor.T = self.saved_pose.copy()
            self.saved_pose = None
            self.runtime.bridge.draw(0.025)
        self.check()
        self.runtime.message = 'Test actor restored. Acknowledge the stop, then Start separately.'


if __name__ == '__main__':
    from Collective_Folder.EnviroParts_S.main_all import main
    main()
