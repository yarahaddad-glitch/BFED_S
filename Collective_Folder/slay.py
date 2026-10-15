from rebot_remote_client import RebotClient
 
def main():

    arm = RebotClient("192.168.50.2")

    try:
        
        arm.enable()
        arm.safe_home()
        arm.move_joints([1.499, -1.1358, -0.5556, -0.5152, -0.0437, -0.0257])
        arm.set_gripper(-4.5008)
        arm.move_joints([1.5036, -1.9034, -0.2684, -1.61, -0.0532, -0.0051])
        print("Pose:", [round(v, 3) for v in arm.tool_pose()])
        arm.move_to_pose(0.018, 0.41+0.02, 0.146, -0.006, 0.014, 1.556)

        # arm.move_joints([1.5223, -1.9293, -0.2714, -1.61, 0.0227, 0.0307])
        arm.set_gripper(-3.0)
        arm.move_joints([0.02, -0.7918, -0.8265, -0.0906, 0.0673, 0.0654])
        arm.move_joints([-2.3116, -0.617, -0.6605, -0.0132, 0.0212, 0.0544])
        arm.move_joints([-2.6049, -2.019, -0.4652, -1.4704, 0.0296, -0.0227])
        arm.set_gripper(-5.0)

        print("Pose:", [round(v, 3) for v in arm.tool_pose()])
        arm.move_to_pose(-0.392, -0.228-0.01, 0.113, -0.03, 0.092, -2.635)

        arm.move_joints([-2.6053, -1.3064, -0.741, -0.4515, -0.1398, -0.0341])
        arm.move_joints([-1.5642, -0.2443, -0.3473, 0.1535, 0.1982, -0.0372])
        arm.safe_home()


        

        # print(arm.set_gripper(-5))
        # print(arm.move_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=5.0))
        # print(arm.set_gripper(0))
        # print(arm.set_gripper(-5))
        # print(arm.move_joints([0.9127, -2.2772, -0.7418, -1.6009, -0.5915, -0.0547], duration=5.0))
        # print(arm.set_gripper(-2.7))
        # print(arm.move_joints([1.1275, -2.1193, -0.7422, -1.4281, -0.0986, -0.0029], duration=5.0))
        # print(arm.move_joints([-1.0779, -1.0714, -0.6594, -0.5476, -0.0948, 0.0189], duration=5.0))
        # print(arm.move_joints([-2.6072, -2.0296, -0.6407, -1.4727, 0.2283, -0.0006], duration=5.0))
        # print(arm.set_gripper(-5))
        # print(arm.move_joints([-2.6061, -1.3636, -0.9524, -0.5842, 0.0513, -0.0780], duration=5.0))
        # print(arm.set_gripper(0))
        # print(arm.move_joints([0.6865, -1.3663, -0.9520, -0.5989, 0.0151, -0.0399], duration=5.0))

    
        
        #print(arm.set_gripper(-2.7))

    finally:

        arm.close()
 
if __name__ == "__main__":

    main()
 