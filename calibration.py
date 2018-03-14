import numpy as np


class Calibration:
    """Stores the transformation relationship between original and target reference frames. The
    class should be instantiated with a minimum of 4 frame->frame calibration points."""
    def __init__(self, frame_a_points, frame_b_points, debug=False):
        # Check data passed into class object
        if type(frame_a_points) is not np.ndarray or type(frame_b_points) is not np.ndarray:
            raise ValueError("Calibration requires data as (n x 3) numpy array.")
        if np.shape(frame_a_points)[1] != 3 or np.shape(frame_b_points)[1] != 3:
            raise ValueError("Array should have only three dimensions.")
        if np.shape(frame_a_points)[0] < 4 or np.shape(frame_b_points)[0] < 4:
            raise ValueError("Array should have at least 4 calibration points.")
        if np.shape(frame_a_points) != np.shape(frame_b_points):
            raise ValueError("Array sizes should match.")

        self.points_from = frame_a_points
        self.points_to = frame_b_points

        number_pts = np.shape(frame_a_points)[0]
        ones = np.ones((number_pts, 1))
        mat_a = np.column_stack([frame_a_points, ones])
        mat_b = np.column_stack([frame_b_points, ones])

        self.transformation = np.linalg.lstsq(mat_a, mat_b, rcond=None)[0]
        self.transformation_reversed = np.linalg.lstsq(mat_b, mat_a, rcond=None)[0]
        if debug:
            print(self.transformation)

    def manual_calibrate(self):
        """Records several camera positions [u,v,w] & end effector positions [x,y,z]
        [u, v, w]*[A] = [x, y, z]
        Return matrix A
        """
        uvw_list = []
        xyz_list = []
        positions = {"camera pos":[uvw_list, self.get_mouth_pos], "end effector position":[xyz_list, self.get_end_effector_pos]}

        while True: 
            n = input("How many points would you like to calibrate with? (<4): ")
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


    def transform(self, coordinate):
        """Transforms an x,y,z coordinate to the target reference frame."""
        if np.shape(coordinate) != (3,):
            raise ValueError("Point must be a (3,) vector.")
        point = np.append(coordinate, 1)
        return np.dot(point, self.transformation)

    def transform_reversed(self, coordinate):
        """Transforms an x,y,z coordinate from the target reference frame back to the original."""
        if np.shape(coordinate) != (3,):
            raise ValueError("Point must be a (3,) vector.")
        point = np.append(coordinate, 1)
        return np.dot(point, self.transformation_reversed)


if __name__ == '__main__':
    #rom numpy import genfromtxt
    #my_data = genfromtxt('cal_data_example.csv', delimiter=',')

    num_pts = 4

    A = np.array([[1.0, 2.0, 3.0],
    [4.0, 5.0, 6.0],
    [7.0, 8.0, 9.0], 
    [2,0, 6.0, 9.0]])
    B = np.array([[1.0, 2.0, 3.0],
    [4.0, 5.0, 6.0], 
    [7.0, 8.0, 9.0], 
    [2,0, 6.0, 9.0]])


    a = A.transpose()
    B = B.transpose()
    print(a)

    print('A', np.shape(A))
    print('B', np.shape(B))

    calibrate = Calibration(A, B)

    sample_in = my_data[0, 0:3]
    sample_out = calibrate.transform(sample_in)
    actual_out = my_data[0, 3:]
    print("Pass in coordinates: ", sample_in)
    print("Convert coordinates: ", sample_out)
    print("Correct coordinates: ", actual_out)