# Grace Chin 2018
#
''' Helps calibrate a point from the camera's frame of reference to the robot's frame of reference
Finds the conversion from finding a list of m values and c values for the different dimensions using y = mx + c
'''
import rospy
import rospkg
import numpy as np
from math import sqrt
from numpy import *
from baxter_control import BaxterControl
from perception_sub import PerceptionSub
from geometry_msgs.msg import Point
from std_msgs.msg import String


class Calibration:
  def __init__(self):
    self.robot = BaxterControl()

  def get_mouth_pos(self):
    '''Gets the position of the mouth detected from the camera
    Returns the point of the mouth'''

    global mouth_sub
    mouth_sub = rospy.Subscriber("/mouth_xyz_astra", Point, self.return_point)
    rospy.wait_for_message("/mouth_xyz_astra", Point)
    return point

  def return_point(self, msg):
    '''Callback function for the mouth subscriber
    Defines the point of the mouth'''
    global point
    point = Point()
    point.x, point.y, point.z = msg.x, msg.y, msg.z
    # print("Camera position:", point)
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

    print("Scale found!")
    print("m list : ", m_list)
    print("c list : ", c_list)
    return scale

  def convert_pt(self, input_pt, scale):
      ''' Converts input_pts using provided scale. 
      input_pts : list containing x, y, z
      scale : list containting m_list and c_list
      Returns output_pt'''

      output_pt = []
      m_list=scale[0]
      c_list=scale[1]

      for di in range(3):
          new_value = input_pt[di]*m_list[di]+c_list[di]
          round_v = math.ceil(new_value * 1000.0) / 1000.0
          output_pt.append(round_v)

      return output_pt


  def manual_calibrate(self):
      """Guided manual calibration in the terminal
      Records several camera positions [u,v,w] & end effector positions [x,y,z]
      Finds a scale containing the list of m and the list of c
      """

      uvw_list = []
      xyz_list = []
      positions = {"camera pos":[uvw_list, self.get_mouth_pos], "end effector position":[xyz_list, self.robot.get_end_effector_pos]}

      while True: 
          n = input("How many points would you like to calibrate with?: ")
          try: 
              n = int(n)
              break
          except:
              print("Please type an integer.")
              pass

      while True:
          # when recording has finished
          if (len(uvw_list)==(n)):
              print("You have finished recording your points.")
              print("Camera coordinates :", uvw_list)
              print("End effector coordinates :", xyz_list)

              print("Applying linear regression...")
              scale = self.linear_regression(uvw_list, xyz_list)
              print("----- Calibration finished -----")

              print("----- Start Testing -----")
              see = raw_input("Would you like to record current camera point? [Y/n]: ")
              if (see == '' or see.lower() == 'y'):
                  camera_point = self.get_mouth_pos() 
                  camera_point = [camera_point.x, camera_point.y, camera_point.z]
                  go = raw_input("Would you like to go to that camera point? [Y/n]: ")
                  print(go)
                  if (go == '' or go.lower() == 'y'):
                      end = self.convert_pt(camera_point, scale)
                      start = self.get_end_effector_pos()
                      print('start:', start)
                      print('end:', end)
                      certain = raw_input("You certain? [Y/n]: ")
                      if (certain == '' or certain.lower() == 'y'):
                          self.robot.set_end_effector_pos(end[0], end[1], end[2])
              return scale

          # recording points
          for pos in positions:
              if (len(positions[pos][0])<(n)):
                  see = raw_input("Would you like to see current "+pos+" value? [Y/n]: ")
                  print(see)
                  if (see == '' or see.lower() == 'y'):
                      see_pos = positions[pos][1]()
                      new_pos = [see_pos.x, see_pos.y, see_pos.z]
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
  rospy.init_node("calibration")
  cal = Calibration()
  cal.manual_calibrate()






  