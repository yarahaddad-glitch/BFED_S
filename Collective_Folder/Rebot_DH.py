from roboticstoolbox import DHRobot, RevoluteDH
from pathlib import Path
from spatialgeometry import Mesh
from spatialmath import SE3
import numpy as np

class ReBot(DHRobot):
    def __init__(self):
        links = [
            RevoluteDH(d=0.055550, a=0.020084, alpha=np.pi/2, offset=0, qlim=[2.8, 2.8]),
            RevoluteDH(d=-0.031625, a=0.264, alpha=np.pi, offset=np.pi, qlim=[-3.14, 0]),
            RevoluteDH(d=-0.0008125, a=0.2485372407, alpha=0, offset=2.9225745762, qlim=[-3.14, 0]),
            RevoluteDH(d=-0.0308125, a=0.078308, alpha=-np.pi/2, offset=0.2190180774, qlim=[-1.87, 1.57]),
            RevoluteDH(d=0.0025, a=0, alpha=np.pi/2, offset=np.pi/2, qlim=[-1.57, 1.57]),
            RevoluteDH(d=0.028008, a=0, alpha=0, offset=0, qlim=[-3.14, 3.14])
        ]

        super().__init__(links, name="ReBot")  #creating DH robot
        self.base = SE3(-0.00008416, 0, 0.08465)  #setting base frame relative to URDF base_link
