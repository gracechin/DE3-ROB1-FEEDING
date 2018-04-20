from numpy import *
from math import sqrt
from baxter_control import BaxterControl
# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

class Calibration:
  def __init__(self):
    self.baxter_control = BaxterControl()

  def __rigid_transform_3D__(self, A, B):
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

  def __transform__(input, R, t):
    output = input*A.T + tile(t, (1, n))
    print output

  def test(self):
    

  def manual_calibrate(self):
          """Records several camera positions [u,v,w] & end effector positions [x,y,z]
          [u, v, w]*[A] = [x, y, z]
          Return matrix A
          """
          uvw_list = []
          xyz_list = []
          positions = {"camera pos":[uvw_list, self.perception.get_rgbd_mouth_pos], "end effector position":[xyz_list, self.arm.get_end_effector_pos]}

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
                  see = raw_input("Record current camera point? [Y/n]: ")
                  if (see == '' or see.lower() == 'y'):
                      camera_point = self.get_mouth_pos() 
                      go = raw_input("Would you like to go to that camera point? [Y/n]: ")
                      print(go)
                      if (go == '' or go.lower() == 'y'):
                          end = self.convert_pt(camera_point, scale)
                          start = self.get_end_effector_pos()
                          print('start:', start)
                          print('end:', end)
                          certain = raw_input("You certain? [Y/n]: ")
                          if (certain == '' or certain.lower() == 'y'):
                              self.arm_ros.move_to(end[0], end[1], end[2], 0.1)
                  return scale

              # recording points
              for pos in positions:
                  if (len(positions[pos][0])<(n)):
                      see = raw_input("See current "+pos+" value? [Y/n]: ")
                      print(see)
                      if (see == '' or see.lower() == 'y'):
                          new_pos = positions[pos][1]()
                          record = raw_input("Record this? [Y/n]: ")
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



  