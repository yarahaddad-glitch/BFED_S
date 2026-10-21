import numpy as np
from spatialmath import SE3
from spatialgeometry import Cuboid
from Collective_Folder import E_STOP

# Zones on the floor: (x_min, y_min, x_max, y_max)
ZONES = {
    "kitchen":   (0, 0, 16.0, 2.5),
    "isolation": (0, 0, 9.4, 24.3),
}
ALLOWED = ["Iso_prisoner1", "Iso_prisoner2"]
PRISONERS = ["prisoner1", "prisoner2", "prisoner3", "Iso_prisoner1", "Iso_prisoner2"]


class PrisonBreak:
    def __init__(self, env, scene, on_trip=None):
        self.env = env
        self.prisoners = {n: scene[n] for n in PRISONERS if n in scene}
        self.muted = set()      # prisoners already reported, until they leave
        self.on_trip = on_trip or E_STOP.stop
        for x0, y0, x1, y1 in ZONES.values():   # show zones as red floor patches
            env.add(Cuboid(scale=[x1 - x0, y1 - y0, 0.02],
                           pose=SE3((x0 + x1) / 2, (y0 + y1) / 2, 0.01),
                           color=[1.0, 0.0, 0.0, 0.25]))

    def arm(self):
        """Check the zones every time env.step() is called."""
        step = self.env.step

        def guarded_step(*args, **kwargs):
            result = step(*args, **kwargs)
            self.check()
            return result

        self.env.step = guarded_step

    def intrusions(self):
        inside = set()
        for name, mesh in self.prisoners.items():
            x, y, z = np.asarray(mesh.T, float)[:3, 3]
            if z < -0.5:
                continue
            for zone, (x0, y0, x1, y1) in ZONES.items():
                if name not in ALLOWED and x0 <= x <= x1 and y0 <= y <= y1:
                    inside.add((name, zone))
        return inside

    def check(self):
        inside = self.intrusions()
        self.muted &= inside
        new = inside - self.muted
        if new:
            for name, zone in new:
                print(f"!!! PRISON BREAK: {name} entered the {zone} area")
            self.on_trip('UNSAFE ZONE: ' + ', '.join(
                f'{name} entered {zone}' for name, zone in sorted(new)))
            self.muted |= inside
        return inside
