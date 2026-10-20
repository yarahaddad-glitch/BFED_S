"""Simulation-only bridge to the existing robot classes. No originals are modified."""
import sys
from pathlib import Path
import numpy as np
from spatialmath import SE3

# .../Assignment 2/Collective_Folder/MealBot_GUI/robot_bridge.py
ASSIGNMENT_DIR = Path(__file__).resolve().parents[2]
if str(ASSIGNMENT_DIR) not in sys.path:
    sys.path.insert(0, str(ASSIGNMENT_DIR))

CONFIG = {
    'MaryamBot': {'module':'Maryam_Work.Maryam_robotclass', 'class':'MaryamBot',
                  'base': lambda: SE3(13.75, 3.5, 0.7) * SE3.Rz(np.pi),
                  'home': [0,0,0,0,0,0]},
    'HannahBot': {'module':'Hannah_Work.Class_HannahBot', 'class':'HannahBot',
                 'base':lambda: SE3(2,0,0), 'home':[0,0,0,0,0,0]},
    'YaraBot': {'module':'Yara_Work.YaraBot_DH', 'class':'YaraBot',
                'base':lambda: SE3(4,0,0), 'home':[0,-90,0,-90,-180,0]},
}

class RobotBridge:
    def __init__(self):
        self.env = None
        self.robots = {}
        self.errors = {}

    def launch(self):
        if self.env is not None:
            return
        import swift
        self.env = swift.Swift()
        self.env.launch(realtime=False)

    def load(self, name):
        if name in self.robots:
            return self.robots[name]
        self.launch()
        from importlib import import_module
        spec = CONFIG[name]
        cls = getattr(import_module(spec['module']), spec['class'])
        robot = cls()
        robot.base = spec['base']()
        robot.q = np.deg2rad(spec['home']).astype(float)
        if name == 'HannahBot':
            robot.add_to_env(self.env)
        else:
            robot.add_to_env(self.env)
        self.robots[name] = robot
        self.env.step(0.01)
        return robot

    def limits_deg(self, name):
        r = self.load(name)
        if name == 'MaryamBot':
            # Use class's conservative limits; same 5-degree margin as RMRC mechanical checks.
            lim = np.asarray(r.qlim, dtype=float)
            conservative = np.asarray(r.safe_qlim, dtype=float)
            result = np.array([np.maximum(lim[0]+np.deg2rad(5),conservative[0]),
                               np.minimum(lim[1]-np.deg2rad(5),conservative[1])])
            return np.rad2deg(result), 'Kawasaki software limits'
        if name == 'HannahBot':
            return np.rad2deg(np.asarray(r.qlim,float)), 'HannahBot class joint limits'
        # Yara class has no qlim: use provisional demo limits, not real/safe manufacturer limits.
        return np.array([[-180,-180,-180,-180,-180,-180],
                         [180,180,180,180,180,180]],float), 'PROVISIONAL demo range: confirm with teammate'

    def angles_deg(self,name):
        return np.rad2deg(self.load(name).q)

    def set_joint(self,name,index,angle_deg):
        robot = self.load(name)
        limits,_ = self.limits_deg(name)
        if not 0 <= index < robot.n or not np.isfinite(angle_deg):
            raise ValueError('Invalid joint command')
        if not limits[0,index] <= angle_deg <= limits[1,index]:
            raise ValueError('Outside configured joint limits')
        q = np.asarray(robot.q,float).copy()
        q[index] = np.deg2rad(angle_deg)
        if name == 'MaryamBot' and not robot.joints_safe(q):
            raise ValueError('Kawasaki software limits reject command')
        # Manual jog is an individual joint setpoint, NOT collision-checked motion planning.
        robot.q = q
        if name == 'HannahBot':
            robot.update_meshes()
        self.env.step(0.01)

    def home(self,name):
        # Preview reset only; not suitable as an automatic post-E-stop motion.
        robot=self.load(name)
        home=np.array(CONFIG[name]['home'],float)
        limits,_=self.limits_deg(name)
        if np.any(home < limits[0]) or np.any(home > limits[1]):
            raise ValueError('Home is outside GUI joint range')
        robot.q = np.deg2rad(home)
        if name=='HannahBot': robot.update_meshes()
        self.env.step(0.01)
