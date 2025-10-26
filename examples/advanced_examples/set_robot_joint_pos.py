from dexcontrol.robot import Robot
import numpy as np

# pose of looking down
head_pos = np.array([-0.01, -0.02078687, 0.00514872])
torso_pos = np.array([7.7998763e-01,  1.5692255e+00, -3.4906584e-04])
left_arm_pos = np.array([0.81562376, 0.3355099, 0.5667102, -2.0946028, 1.270783, -0.01205499, -0.00896401])
right_arm_pos = np.array([ 0.23443438, -0.29044896,  0.04472581, -2.183742,   -1.2762475,  -0.03269351, -0.04249527])



with Robot() as bot:
    # left_arm_pos = bot.left_arm.get_predefined_pose("folded")
    # right_arm_pos = bot.right_arm.get_predefined_pose("folded")
    torso_pos = bot.torso.get_predefined_pose("folded")
    bot.torso.set_joint_pos(torso_pos, wait_time=10.0, exit_on_reach=True)
    # bot.set_joint_pos(
    #     {
    #         "head": head_pos,
    #         "torso": torso_pos,
    #         "left_arm": left_arm_pos,
    #         "right_arm": right_arm_pos,
    #     },
    #     wait_time=20.0,
    # )
    
    # bot.head.set_joint_pos(head_pos, wait_time=2.0, exit_on_reach=True)