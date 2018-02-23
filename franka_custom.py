"""Python Module to control the Franka Arm though simple method calls.

This module uses ``subprocess`` and ``os``.
"""
import os
import sys
import subprocess
import __future__
import rospy
from geometry_msgs.msg import Point
from franka.franka_control import FrankaControl
from astra import MouthPos
import numpy as np
from itertools import combinations

global arm 
arm = FrankaControl()

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

    def get_end_effector_pos(self):
        xyz_pos = arm.get_end_effector_pos()
        print("End effector position:", xyz_pos) 
        return xyz_pos

    def get_mouth_pos(self):
        rospy.init_node("FredNode", anonymous=True)
        global mouth_sub
        mouth_sub = rospy.Subscriber("mouthxyz", Point, self.return_point)
        rospy.wait_for_message("mouthxyz", Point)
        return point

    def return_point(self, msg):
        global point
        point = [msg.x, msg.y, msg.z]
        print("Camera position:", point)
        mouth_sub.unregister()
        return

    def linear_regression(self, set1, set2):
        ''' Finds the relationship between two sets of points (y=mx+c)
        Inputs two lists: input list (set1) and output list (set2)
        Returns list of m and c.'''
        A = np.array(set1)
        B = np.array(set2)

        m_list = [] # m for x, y, z
        c_list = [] # c for x, y, z

        for dimension in range(3):
            a = []
            b = []
            for point_index in range(len(A)):
                a.append(A[point_index][dimension])
                b.append(B[point_index][dimension])
            np.array(a)
            np.array(b)
            a = np.vstack([a, np.ones(len(a))]).T
            m, c = np.linalg.lstsq(a, b)[0]
            m_list.append(m)
            c_list.append(c)
            scale=[m_list, c_list]
        return scale

    def convert_pt(self, input_pts, scale):
        ''' Converts input_pts using provided scale. 
        input_pts : list containing x, y, z
        scale : list containting m_list and c_list
        Returns output_pt'''
        output_pt = []
        m_list=scale[0]
        c_list=scale[1]
        for point in range(len(input_pts)):
            out = []
            for di in range(3):
                output_pt.append(input_pts[point][di]*m_list[di]+c_list[di])
            output_pt.append(out)
        return ouput_pt


    def calibrate(self):
        """Records several camera positions [u,v,w] & end effector positions [x,y,z]
        [u, v, w]*[A] = [x, y, z]
        Return matrix A
        """
        uvw_list = []
        xyz_list = []
        positions = {"camera pos":[uvw_list, self.get_mouth_pos], "end effector position":[xyz_list, self.get_end_effector_pos]}

        while True: 
            n = input("How many points would you like to calibrate with?: ")
            try: 
                n = int(n)
                break
            except:
                print("Please type an integer.")
                pass

        while True:
            # recording finished
            if (len(uvw_list)==(n)):
                print("You have finished recording your points.")
                print("Camera coordinates :", uvw_list)
                print("End effector coordinates :", xyz_list)

                print("Applying linear regression...")
                m_list, c_list = self.linear_regression(uvw_list, xyz_list)
                return X

            # recording points
            for pos in positions:
                if (len(positions[pos][0])<(n)):
                    see = raw_input("Would you like to see current "+pos+" value? [Y/n]: ")
                    print(see)
                    if (see == '' or see.lower() == 'y'):
                        new_pos = positions[pos][1]()
                        record = raw_input("Would you like to record this? [Y/n]: ")
                        if (record == '' or record.lower() == 'y'):
                            positions[pos][0].append(new_pos)
                        elif (record.lower() == 'n'): pass
                        elif (record.lower() == 'q'): sys.quit()
                        else: print("Invalid response.")
                    elif (see.lower() == 'n'): pass
                    else: print("Invalid response.")
                    print("Camera coordinates :", uvw_list)
                    print("End effector coordinates :", xyz_list)
                else:
                    pass


if __name__ == '__main__':
    main()