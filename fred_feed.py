# Importing Chess team - Ben Greenberg's classes
# from franka.franka_control import FrankaControl
from franka.franka_control_ros import FrankaRos
# from Franka import FrankaCustom
from time import sleep


# initiating FrankaControl
# arm = FrankaControl(debug_flag=True) 
arm_ros = FrankaRos()
# fred = FrankaCustom()


arm_ros.move_gripper(0.03, 0.1)
arm_ros.grasp(0.06, 0.1, 0)

# print(arm_ros.get_joint_positions())
# sleep(5)
# arm_ros.move_to(0.62467, 0.0376194, 0.405594, 0.1)
# print(arm_ros.get_joint_positions())
# sleep(5)
# arm_ros.move_to(0.662691,0.0355911,0.000569615, 0.1)
# print(arm_ros.get_joint_positions())
# sleep(5)
# arm_ros.move_to(0.62467, 0.0376194, 0.405594, 0.1)
# print(arm_ros.get_joint_positions())
# sleep(5)