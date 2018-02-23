import subprocess
from std_msgs.msg import String
from franka_custom import FrankaCustom
from franka.franka_control import FrankaControl
from astra import MouthPos
import matplotlib.pyplot as plt
import numpy as np

arm = FrankaControl(debug_flag=True) 

print(arm.get_joint_positions())
arm.move_relative(-0.1, 0, 0)