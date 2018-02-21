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

    def linear_regression(self, a, b):
        """Finds the matrix X, such that A*X = B
            Adapted from: https://stackoverflow.com/questions/33559946/numpy-vs-mldivide-matlab-operator
        Return matrix X
        """

        # converting list to matrix
        A = np.matrix(a)
        B = np.matrix(b)

        # applying python version of \ (mldivide(A, B)) in Matlab X = A\B --> A*X=B
        num_vars = A.shape[1]
        rank = np.linalg.matrix_rank(A)
        if rank == num_vars:              
            X = np.linalg.lstsq(A, B)[0]    # not under-determined
        else:
            for nz in combinations(range(num_vars), rank):    # the variables not set to zero
                try: 
                    X = np.zeros((num_vars, 1))  
                    X[nz, :] = np.asarray(np.linalg.solve(A[:, nz], B))
                    #print(X)
                except np.linalg.LinAlgError:     
                    pass                    # picked bad variables, can't solve

        # checking X
        if (np.array_equal(A*X,B)):
            print("calibration factor (X) is found.")
            print("calibration completed.")
        else:
            print("calibration error!")
            print("A:", A, "B:", B, "X:", X)

        return X


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
                X = self.linear_regression(self, uvw_list, xyz_list)
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