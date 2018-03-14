Using Franka
============

Setting Up
^^^^^^^^^^

Visit https://de3-rob1-chess.readthedocs.io to see how franka was setted up.

This `site <https://frankaemika.github.io>`_ was used to install the live kernel, libfranka and franka_ros.

Running Baxter
^^^^^^^^^^^^^^

The Baxter Robot can be started using the provided lab computers. 

All codes are stored in the ''catkin_ws'' folder in the computer.
The executable files are in the ''src'' folder. The folders inside the ''src'' folder are packages.
To connect to Baxter, ensure that your IP address is recorded in ''baxter.sh''.

To start Baxter: 

.. code-block:: python

	$ cd ~/catkin_ws
	$ bash baxter.sh 

To run a python program:

$ rosrun {name_of_package} {python_filename}

Replace the content in the “{ …}” with the relevant text. 
To make an executable python file, each python file starts with:

#!/usr/bin/env python

Modules are also imported as shown below:

import rospy
import rospkg
import baxter_interface
import baxter_external_devices
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

If you would like to move the arm, run the following to enable the arms in the terminal:

$ rosrun baxter_tools enable_robot.py -e

To disable the arms and prevent overheating, or when it is not in use, change  -e  to  -d. 


Controlling Baxter
^^^^^^^^^^^^^^^^^^

FRANKA can be controlled using the libfranka library, which is only accessible using the C++ language. Petar and Fabian have made some libfranka functions accessible by Python and the Chess team has made a Python library for FRANKA.




Credits 
^^^^^^^
Felix in the DeVito team for the crash course on how to use Baxter. 
