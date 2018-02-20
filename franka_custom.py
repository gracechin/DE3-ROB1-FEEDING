"""Python Module to control the Franka Arm though simple method calls.

This module uses ``subprocess`` and ``os``.
"""

import __future__
from franka.franka_control import FrankaControl
import subprocess
import os


class FrankaCustom:
    """Class containing methods to control an instance of the Franka Arm.

    Will print debug information to the console when ``debug_flag=True`` argument is used. Class
    references C++ binaries to control the Franka.

    IP address of Franka in Robotics Lab already configured as default.
    """
    def __init__(self, ip='192.168.0.88', debug_flag=False):
        self.ip_address = ip
        self.debug = debug_flag
        self.path = os.path.dirname(os.path.realpath(__file__))  # gets working dir of this file


    def mouth_pos(self, callback_func, loop_tf = True):
        """Gets mouth position

        Return mouth position: list [u, v, w]
        """
        return

    def uvw_xyz_calibration(self):
        """Finds the matrix A, such that ([u, v, w]')*[A] --> [x, y, z]'

        Return matrix A
        """
        return



# def main():
#     """Used to test if module is working and can control arm.
#
#     When ``franka.py`` is run from the command line it will test to see if the Franka Arm can be
#     controlled with a simple forward and backward motion control along the x axis. Follow on
#     screen examples for usage."""
#
#     while True:
#         testing = input("Is this program being tested with the arm? [N/y]: ")
#         if testing == '' or testing.lower() == 'n':
#             testing = False
#             break
#         elif testing.lower() == 'y':
#             testing = True
#             break
#         else:
#             print("Invalid response.")
#     print("Testing mode: ", testing)
#
#     while True:
#         direction = input("Enter 0 to move along x slightly, 1 for backwards: ")
#         if direction in ['0', '1']:
#             break
#         else:
#             print("Invalid input. Must be 0/1.")
#
#     if testing:
#         arm = FrankaControl(debug_flag=True)
#
#         if direction == '0':
#             arm.move_relative(dx=0.05)
#         elif direction == '1':
#             arm.move_relative(dx=-0.05)
#
#     else:
#         dx = '0'
#         dy = '0'
#         dz = '0'
#         if direction == '0':
#             dx = 0.05
#         elif direction == '1':
#             dx = -0.05
#         print("dx: ", dx)
#         print("dy: ", dy)
#         print("dz: ", dz)
#
#         program = './franka_move_to_relative'
#         ip_address = '192.168.0.88'
#
#         print("Program being run is: ", program)
#         print("IP Address of robot: ", ip_address)
#
#         command = [program, ip_address, dx, dy, dz]
#         command_str = " ".join(command)
#
#         print("Command being called: ", command_str)


if __name__ == '__main__':
    main()