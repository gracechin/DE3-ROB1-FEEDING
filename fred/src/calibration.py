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
  # self.camera = PerceptionSub()

  def __rigid_transform_3D__(self, A, B):
    # Input: expects Nx3 matrix of points
    # Returns R,t
    # R = 3x3 rotation matrix
    # t = 3x1 column vector
    print('A:', A)
    print('B:', B)
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)
    
    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = transpose(AA) * BB

    U, S, Vt = linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T
    return R, t

  # def convert_pt(self, input_mat, R, t):
  #   print('R', R)
  #   print('t', t)
  #   n = input_mat.shape[0]
  #   output_mat = R*input_mat.T + tile(t, (1, n))
  #   print(np.shape(output_mat)) 
  #   output_mat = output_mat.tolist()
  #   output = [output_mat[0][0], output_mat[1][0], output_mat[2][0]]
  #   print(output)
  #   return output

  def test(self):
    # Test with random data
    # Random rotation and translation
    R = mat(random.rand(3,3))
    t = mat(random.rand(3,1))

    # make R a proper rotation matrix, force orthonormal
    U, S, Vt = linalg.svd(R)
    R = U*Vt

    # remove reflection
    if linalg.det(R) < 0:
       Vt[2,:] *= -1
       R = U*Vt

    # number of points
    n = 10

    A = mat(random.rand(n,3));
    # how to transform A to B with matrix R and vector 
    B = R*A.T + tile(t, (1, n))
    B = B.T;

    # recover the transformation
    ret_R, ret_t = self.__rigid_transform_3D__(A, B)

    A2 = (ret_R*A.T) + tile(ret_t, (1, n))
    A2 = A2.T

    # Find the error
    err = A2 - B

    err = multiply(err, err)
    err = sum(err)
    rmse = sqrt(err/n);

    print "Points A"
    print A
    print ""

    print "Points B"
    print B
    print ""

    print "Rotation"
    print R
    print ""

    print "Translation"
    print t
    print ""

    print "RMSE:", rmse
    print "If RMSE is near zero, the function is correct!"

  def get_mouth_pos(self):
    global mouth_sub
    mouth_sub = rospy.Subscriber("/mouth_xyz_astra", Point, self.return_point)
    rospy.wait_for_message("/mouth_xyz_astra", Point)
    return point

  def return_point(self, msg):
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

    #print("set1", set1)
    #print("set2", set2)
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
      """Records several camera positions [u,v,w] & end effector positions [x,y,z]
      [u, v, w]*[A] = [x, y, z]
      Return matrix A
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
    

  # def calibrate_robot_camera(self):
  #   """Records several camera positions [u,v,w] & end effector positions [x,y,z]
  #   [u, v, w]*[A] = [x, y, z]
  #   Return matrix A
  #   """
  #   uvw_list = []
  #   xyz_list = []
  #   positions = {"camera pos":[uvw_list, self.get_mouth_pos], "end effector position":[xyz_list, self.robot.get_end_effector_pos]}

  #   while True: 
  #       n = input("How many points would you like to calibrate with?: ")
  #       try: 
  #           n = int(n)
  #           break
  #       except:
  #           print("Please type an integer.")
  #           pass

  #   while True:
  #       # when recording has finished
  #       if (len(uvw_list)==(n)):
  #           print("You have finished recording your points.")
  #           print("Applying roni's calibration code")
  #           uvw_mat = np.matrix(uvw_list)
  #           xyz_mat = np.matrix(xyz_list)
  #           R, t = self.__rigid_transform_3D__(uvw_mat, xyz_mat)
  #           print("----- Calibration finished -----")

  #           print("----- Start Testing -----")
  #           camera_pt = self.get_mouth_pos() 
  #           print('camera pt:', camera_pt)
  #           see = raw_input("Record current camera point? [Y/n]: ")
  #           if (see == '' or see.lower() == 'y'):
  #               camera_mat = np.matrix([camera_pt.x, camera_pt.y, camera_pt.z])
  #               print('camera mat:', camera_mat)
  #               go = raw_input("Go to camera point? [Y/n]: ")
  #               print(go)
  #               if (go == '' or go.lower() == 'y'):
  #                   end = self.convert_pt(camera_mat, R, t)
  #                   start_pt = self.robot.get_end_effector_pos()
  #                   start = [start_pt.x, start_pt.y, start_pt.z]
  #                   print('start:', start)
  #                   print('end:', end)
  #                   certain = raw_input("You certain? [Y/n]: ")
  #                   if (certain == '' or certain.lower() == 'y'):
  #                       self.robot.set_end_effector_pos(end[0], end[1], end[2])
  #           return R, t

  #       # recording points
  #       for pos in positions:
  #           if (len(positions[pos][0])<(n)):
  #               see = raw_input("See current "+pos+" value? [Y/n]: ")
  #               print(see)
  #               if (see == '' or see.lower() == 'y'):
  #                   see_pos = positions[pos][1]()
  #                   new_pos = [see_pos.x, see_pos.y, see_pos.z]
  #                   print(new_pos)
  #                   record = raw_input("Record this? [Y/n]: ")
  #                   if (record == '' or record.lower() == 'y'):
  #                       positions[pos][0].append(new_pos)
  #                   elif (record.lower() == 'n'): pass
  #                   elif (record.lower() == 'q'): sys.quit()
  #                   else: print("Invalid response.")
  #               elif (see.lower() == 'n'): pass
  #               else: print("Invalid response.")
  #               print("Camera coordinates :"+str(len(uvw_list))+' out of '+str(n)) 
  #               print("End effector coordinates :"+str(len(xyz_list))+' out of '+str(n))
  #           else:
  #               pass

if __name__ == '__main__':
  rospy.init_node("calibration")
  cal = Calibration()
  # input1 = np.array([[ 272.,  227.,   37.]])
  # R = np.matrix([[ 0.        , -0.8479665 , -0.53004982],
  #       [-0.88769737,  0.24404946, -0.390427  ],
  #       [ 0.46042739,  0.47052383, -0.75273763]])
  # t = np.matrix([[ 214.64096768],
  #       [ 201.82389656],
  #       [-200.41309383]])
  # cal.convert_pt(input1, R, t)


  rospy.init_node("calibration")
  cal = Calibration()
  cal.manual_calibrate()

  
  # m_list = [0.00015460745105730597, -0.0049997272258064098, -0.00068621159500533184]
  # c_list = [0.97864597698655964, 0.82640173090991298, 0.88186721651139377]
  # scale=[m_list, c_list]
  # see = raw_input("Would you like to record current camera point? [Y/n]: ")
  # if (see == '' or see.lower() == 'y'):
  #     camera_point = cal.get_mouth_pos() 
  #     camera_point = [camera_point.x, camera_point.y, camera_point.z]
  #     go = raw_input("Would you like to go to that camera point? [Y/n]: ")
  #     print(go)
  #     if (go == '' or go.lower() == 'y'):
  #         end = cal.convert_pt(camera_point, scale)
  #         start = cal.robot.get_end_effector_pos()
  #         print('start:', start)
  #         print('end:', end)
  #         certain = raw_input("You certain? [Y/n]: ")
  #         if (certain == '' or certain.lower() == 'y'):
  #             cal.robot.set_end_effector_pos(end[0], end[1], end[2])





  