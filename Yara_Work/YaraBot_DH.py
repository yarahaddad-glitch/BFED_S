import os
from math import pi

import roboticstoolbox as rtb

from ir_support.robots.UTSMeshRobot import UTSMeshRobot

class YaraBot(UTSMeshRobot):
    def __init__(self, base=None):
        #creat the DH links
        links = self._create_DH()

        #folder containing the DAE mesh files 
        mesh_directory = os.path.join(os.path.abspath(os.path.dirname(__file__)), "EC66_Mesh")

        #intitialisng the utsMeshRobot 

        super().__init__(links=links,
                         mesh_stem = "EC66",
                         mesh_dir = mesh_directory,
                         name = "YaraBot",
                         home_q = [0, 0, 0, 0, 0, 0],
                         base=base,
                         meshes_are_global_at_home=True,
                        )

    def _create_DH(self):

        return[
        rtb.RevoluteDH(d=0.096, a=0, alpha= -pi/2),
        rtb.RevoluteDH(d=0, a=0.418, alpha= 0),
        rtb.RevoluteDH(d=0, a=0.398, alpha= 0),
        rtb.RevoluteDH(d=0.122, a=0, alpha= -pi/2),
        rtb.RevoluteDH(d=0.098, a=0, alpha= -pi/2),
        rtb.RevoluteDH(d=0.089, a=0, alpha= 0),
        ]
