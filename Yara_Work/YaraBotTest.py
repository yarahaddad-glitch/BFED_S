import swift
from spatialgeometry import Mesh
from spatialmath import SE3
import keyboard
import time

#start swift simulator
env = swift.Swift()
env.launch(realtime=True, browser=None)

#load the robot mesh files:
Base = Mesh(filename= "BFED_S/Yara_Work/EC66_Mesh/EC66Link0.dae")
Joint1 = Mesh(filename= "BFED_S/Yara_Work/EC66_Mesh/EC66Link1.dae")
Joint2 = Mesh(filename= "BFED_S/Yara_Work/EC66_Mesh/EC66Link2.dae")
Joint3 = Mesh(filename= "BFED_S/Yara_Work/EC66_Mesh/EC66Link3.dae")
Joint4 = Mesh(filename= "BFED_S/Yara_Work/EC66_Mesh/EC66Link4.dae")
Joint5 = Mesh(filename= "BFED_S/Yara_Work/EC66_Mesh/EC66Link5.dae")
Joint6 = Mesh(filename= "BFED_S/Yara_Work/EC66_Mesh/EC66Link6.dae")

#add the mesh files to swift:
env.add(Base)
env.add(Joint1)
env.add(Joint2)
env.add(Joint3)
env.add(Joint4)
env.add(Joint5)
env.add(Joint6)

env.step()

input("press enter to end")
