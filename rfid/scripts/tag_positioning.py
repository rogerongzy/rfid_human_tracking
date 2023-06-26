import cv2
import rospy
import random
import apriltag
import numpy as np
import pyrealsense2 as rs
from cmath import exp
from rfid.msg import rfid_msg
import math
import csv
import matplotlib.pyplot as plt


################### modified by Rong Zhiyi ####################

################## embedded in a config file ##################
# TAG_ID = 'E280-1160-6000-0209-F811-48C3' # white, apriltag
TAG_ID = 'E280-1160-6000-0209-F811-5C03' # brown, aruco
PHASE_LOAD_NUM = 4
TAG_SIZE = 0.16
# antenna params
NUM_ANTENNA = 4
# WAVE_LENGTH = 0.232924707 # tuned
WAVE_LENGTH = 0.162924707 * 2 # provided
# camera params
F_X, F_Y = 649.376, 649.376 # focal_length
C_X, C_Y = 648.137, 353.517 # principal_point
# structural params
X_OFFSET = 0.032
Y_OFFSET = 0.085
Z_OFFSET = 0.06
HALF_SIDE = 0.125
# candidates params
RADIUS = 0.05 # metre
NUM_CANDIDATES = 100
####################### saving data ###########################
phase_dist_file = open('phase_dist.csv', 'a+')
phase_dist_writer = csv.writer(phase_dist_file)
traj_gt_file = open('traj_gt.csv', 'a+')
traj_gt_writer = csv.writer(traj_gt_file)
traj_pd_file = open('traj_pd.csv', 'a+')
traj_pd_writer = csv.writer(traj_pd_file)







###############################################################
# considering the single antenna first
# phase_loader consists of 4 consecutive phase (can be interrupted)

class RFID_Subscriber:
    def __init__(self):
        ### camera
        # self.camera_init()

        ### callback_1, multi-aperture version
        # self.phasor_unwrapped = []
        # self.phase_old = None # None, 0
        # self.phase_current = None # None, 0

        ### callback_2, parallel version
        self.phasor_unwrapped = np.array([None] * NUM_ANTENNA)
        self.phasor_old = np.array([None] * NUM_ANTENNA)
        self.phasor_current = np.array([None] * NUM_ANTENNA)

        ### ROS
        self.subscriber = rospy.Subscriber('/rfid_message', rfid_msg, self.callback_2) # choose callback 1 or 2
        self.action_timestamp = None # used to adjust the processing period

        self.location_current = np.array([-0.5, 0, 1]) # starting point used for manual-grid-calibration


    # def camera_init(self):
    #     self.pipeline = rs.pipeline()
    #     self.config = rs.config()
    #     # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) # depth
    #     self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) # color
    #     self.pipeline.start(self.config)
    #     self.tag_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
    #     self.tag_detect() # initialize tag_camera_position
    #     # self.location_current = np.array([self.tag_camera_position[0], self.tag_camera_position[1], self.tag_camera_position[2]]) # used for calibration
        

    # def tag_detect(self):
    #     frames = self.pipeline.wait_for_frames()
    #     # depth_frame = frames.get_depth_frame()
    #     color_frame = frames.get_color_frame()
    #     # depth_image = np.asanyarray(depth_frame.get_data()) # numpy.ndarray
    #     color_image = np.asanyarray(color_frame.get_data())
    #     gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
    #     # show images, only for illustration
    #     # cv2.imshow('RealSense', color_image)
    #     # key = cv2.waitKey(1)

    #     tags_list = self.tag_detector.detect(gray)
    #     for tags in tags_list:
    #         # add selectioin on tags (environment disturbance exists)
    #         if tags.tag_id == 0:
    #             pose, _, _ = self.tag_detector.detection_pose(tags, camera_params=(F_X, F_Y, C_X, C_Y), tag_size=TAG_SIZE)
    #             self.tag_camera_position = pose[:3, 3]
    #             # rotation = pose[:3, :3]


    def tag_antenna_position(self, tag_refpt_position, antenna_id):
        position = np.array([tag_refpt_position[0], tag_refpt_position[1], tag_refpt_position[2]])
        # calibration using camera rgb stream, reference point is rgb camera (left1)
        # position[0] = position[0] + (2 * antenna_id - 5) * HALF_SIDE - X_OFFSET # 1:-3 2:-1 3:+1 4:+3 
        # position[1] = position[1] - Y_OFFSET - HALF_SIDE
        # position[2] = position[2] + Z_OFFSET
        # calibration using manual grid, reference point is antenna centre
        position[0] = position[0] + (2 * antenna_id - 5) * HALF_SIDE
        return position


    # def camera_shutdown(self):
    #     self.pipeline.stop()


    # multiple apertures version (single antenna)
    # def callback_1(self, msg):
    #     ########################################
    #     ############# message type #############
    #     # epc: "E280-1160-6000-0209-F811-48C3"
    #     # time: 1684307980026460
    #     # idx: 3
    #     # mode: 2
    #     # ant: 4
    #     # phase: 333.28125
    #     # rssi: -59.0
    #     ########################################
    #     ########################################
    #     # considering the single antenna case
    #     if msg.epc == TAG_ID and msg.ant == 1:
    #         if len(self.phasor_unwrapped) == 0:
    #             self.phasor_unwrapped.append(msg.phase / 180 * np.pi)
    #             self.phase_old = msg.phase / 180 * np.pi
    #         else:
    #             self.phase_current = msg.phase / 180 * np.pi
    #             if self.phase_current - self.phase_old > np.pi:
    #                 self.phasor_unwrapped.append(self.phasor_unwrapped[-1] + self.phase_current - self.phase_old - 2 * np.pi)
    #             elif self.phase_old - self.phase_current > np.pi:
    #                 self.phasor_unwrapped.append(self.phasor_unwrapped[-1] + self.phase_current - self.phase_old + 2 * np.pi)
    #             else:
    #                 self.phasor_unwrapped.append(self.phasor_unwrapped[-1] + self.phase_current - self.phase_old)

    #             if len(self.phasor_unwrapped) > PHASE_LOAD_NUM:
    #                 del self.phasor_unwrapped[0] # phasor_unwrapped.pop(0)
    #             self.phase_old = self.phase_current

    #             # recording phase in .txt file
    #             # file2.write(str(self.phasor_unwrapped[-1]) + "\n")
    #             # file1.write(str(self.phase_current) + "\n")
                
    #         print('-------------------------------')
    #         print(self.phasor_unwrapped)
    #         # print(self.phasor_normalized(self.phasor_unwrapped))

    #         self.tag_detect()

    #         print(self.tag_camera_position)
    #         print(self.tag_antenna_position(self.tag_camera_position, msg.ant))
    #         print(np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant)))

    #         # file3.write(str(np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant))) + "\n") # true euclidean distance
    #         # file3.write(str(self.tag_camera_position[2]) + "\n") # distance along z axis



    # multiple antennas (parallel)
    def callback_2(self, msg):

        if msg.epc == TAG_ID:

            if self.phasor_unwrapped[msg.ant - 1] == None:
            
                # original
                # self.phasor_unwrapped[msg.ant - 1] = msg.phase / 180 * np.pi
                # self.phasor_old[msg.ant - 1] = msg.phase / 180 * np.pi
                
                # modified by introducing initial calibration
                # self.phasor_unwrapped[msg.ant - 1] = np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant)) * 4 * np.pi / WAVE_LENGTH # calibration using camera rgb stream
                self.phasor_unwrapped[msg.ant - 1] =  np.linalg.norm(self.tag_antenna_position(self.location_current, msg.ant)) * 4 * np.pi / WAVE_LENGTH # calibration using manual grid

                # self.phasor_unwrapped[msg.ant - 1] = -msg.phase / 180 * np.pi
                self.phasor_old[msg.ant - 1] = msg.phase / 180 * np.pi
                

                # modified_unwrapped_phase.append(-float(line2) + float(data2[0]) + float(data3[0]) * 4 * np.pi / lmd)
                # np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant))

                self.action_timestamp = msg.time

            else:
                # original
                # self.phasor_current[msg.ant - 1] = msg.phase / 180 * np.pi

                # if self.phasor_current[msg.ant - 1] - self.phasor_old[msg.ant - 1] > np.pi:
                #     self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] + self.phasor_current[msg.ant - 1] - self.phasor_old[msg.ant - 1] - 2 * np.pi
                # elif self.phasor_old[msg.ant - 1] - self.phasor_current[msg.ant - 1] > np.pi:
                #     self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] + self.phasor_current[msg.ant - 1] - self.phasor_old[msg.ant - 1] + 2 * np.pi
                # else:
                #     self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] + self.phasor_current[msg.ant - 1] - self.phasor_old[msg.ant - 1]
            
                # self.phasor_old[msg.ant - 1] = self.phasor_current[msg.ant - 1]
                

                # modified by real-time initial calibration
                self.phasor_current[msg.ant - 1] = msg.phase / 180 * np.pi

                if self.phasor_current[msg.ant - 1] - self.phasor_old[msg.ant - 1] > np.pi:
                    self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] - self.phasor_current[msg.ant - 1] + self.phasor_old[msg.ant - 1] + 2 * np.pi
                elif self.phasor_old[msg.ant - 1] - self.phasor_current[msg.ant - 1] > np.pi:
                    self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] - self.phasor_current[msg.ant - 1] + self.phasor_old[msg.ant - 1] - 2 * np.pi
                else:
                    self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] - self.phasor_current[msg.ant - 1] + self.phasor_old[msg.ant - 1]
            
                self.phasor_old[msg.ant - 1] = self.phasor_current[msg.ant - 1]

                ## segmentation used to record data from 4 antenna ###
                # if msg.ant == 1:
                #     file1.write(str(self.phasor_unwrapped[msg.ant - 1]) + "\n")
                #     file5.write(str(np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant)) * 4 * np.pi / WAVE_LENGTH) + "\n")
                # elif msg.ant == 2:
                #     file2.write(str(self.phasor_unwrapped[msg.ant - 1]) + "\n")
                #     file6.write(str(np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant)) * 4 * np.pi / WAVE_LENGTH) + "\n")
                # elif msg.ant == 3:
                #     file3.write(str(self.phasor_unwrapped[msg.ant - 1]) + "\n")
                #     file7.write(str(np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant)) * 4 * np.pi / WAVE_LENGTH) + "\n")
                # elif msg.ant == 4:
                #     file4.write(str(self.phasor_unwrapped[msg.ant - 1]) + "\n")
                #     file8.write(str(np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant)) * 4 * np.pi / WAVE_LENGTH) + "\n")
                    
                #     self.tag_detect()
                #     traj_gt_writer.writerow([self.tag_camera_position[0], self.tag_camera_position[1], self.tag_camera_position[2]])

                if msg.ant == 4:
                    # self.tag_detect() # messages for 4 antennas can be seen simultaneous, where detection only need to be done once
                    
                    # sampler by time, x seconds
                    # if msg.time - self.action_timestamp > 0.2 * 1000000:
                    #     self.action_timestamp = msg.time
                    
                    # no time sampling
                    if True:

                        ### process indicator ###
                        print('-------------------------------------')
                        print(msg.time)

                        ### updating the current cordinate with the most possible candidate (complete process)
                        # predicted_coordinate = self.candidates_generator()
                        # self.location_current = predicted_coordinate
                        
                        
                        ### writer for trajectory, both ground truth and predicted ###
                        # traj_gt_writer.writerow([self.tag_camera_position[0], self.tag_camera_position[1], self.tag_camera_position[2]])
                        # traj_pd_writer.writerow([predicted_coordinate[0], predicted_coordinate[1], predicted_coordinate[2]])

                        ### writer for phase and distance ###
                        # phase_dist_writer.writerow([self.phasor_unwrapped[0], self.phasor_unwrapped[1], self.phasor_unwrapped[2], self.phasor_unwrapped[3], np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, 1)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, 2)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, 3)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, 4)) * 4 * np.pi / WAVE_LENGTH])
                        phase_dist_writer.writerow([self.phasor_unwrapped[0], self.phasor_unwrapped[1], self.phasor_unwrapped[2], self.phasor_unwrapped[3], 0, 0, 0, 0])
                        






                    # print('-------------------------------')
                    # print(self.phasor_unwrapped)

                    # print(self.phasor_normalized(self.phasor_unwrapped))
                    # print(np.linalg.norm(self.phasor_normalized(self.phasor_unwrapped)))
                    # print(self.phasor_normalized(self.phasor_unwrapped).getH())
                    # print(np.linalg.norm(self.phasor_normalized(self.phasor_unwrapped).getH()))
                    # print(self.cosine_similarity(self.phasor_normalized(self.phasor_unwrapped), self.phasor_normalized(self.phasor_unwrapped)))


                    # print(self.location_current)
                    # print(self.tag_camera_position)
                    # print(self.tag_antenna_position(self.tag_camera_position, msg.ant))

                    







    def generate_random_coordinate(self, centre, radius):
        random_radius = random.uniform(0, radius)
        random_angle = random.uniform(0, 2 * np.pi)
        new_coordinate = np.array([centre[0] +  random_radius * np.cos(random_angle), centre[1], centre[2] + random_radius * np.sin(random_angle)])
        return new_coordinate


    def phasor_normalized(self, phasor):
        phasor_temp = []
        for i in range(len(phasor)):
            phasor_temp.append(exp(-1j*(phasor[i] - phasor[0])))
        # phasor_normalized = np.matrix(phasor_temp)
        phasor_normalized = np.array(phasor_temp)
        return phasor_normalized


    def cosine_similarity(self, complex_vec_1, complex_vec_2):
        # num = np.linalg.norm(np.dot(complex_vec_1, complex_vec_2.getH()))
        num = np.linalg.norm(np.dot(complex_vec_1, complex_vec_2.conjugate().T))
        # num = np.dot(complex_vec_1, complex_vec_2.getH())
        # print(num)
        den = np.linalg.norm(complex_vec_1) * np.linalg.norm(complex_vec_2)
        # print(den)
        return num / den


    def candidates_generator(self):
        # only in 2D top-view plane
        candidates_list = []
        result_list = []
        for i in range(NUM_CANDIDATES):

            # temp_coordinate = np.array([self.location_current[0] + RADIUS * np.cos(i * 2 * np.pi / NUM_CANDIDATES), self.location_current[1], self.location_current[2] + RADIUS * np.sin(i * 2 * np.pi / NUM_CANDIDATES)])
            temp_coordinate = self.generate_random_coordinate(self.location_current, RADIUS) # invariant radius, modified into heuristic

            temp_phasor = np.array([np.linalg.norm(self.tag_antenna_position(temp_coordinate, 1)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 2)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 3)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 4)) * 4 * np.pi / WAVE_LENGTH])
            temp_similarity = self.cosine_similarity(self.phasor_normalized(self.phasor_unwrapped), self.phasor_normalized(temp_phasor))

            candidates_list.append(temp_coordinate)
            result_list.append(temp_similarity)

        ## if random, no need to include itself
        # temp_coordinate = np.array([self.location_current[0], self.location_current[1], self.location_current[2]])
        # temp_phasor = np.array([np.linalg.norm(self.tag_antenna_position(temp_coordinate, 1)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 2)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 3)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 4)) * 4 * np.pi / WAVE_LENGTH])
        # temp_similarity = self.cosine_similarity(self.phasor_normalized(self.phasor_unwrapped), self.phasor_normalized(temp_phasor))
        # candidates_list.append(temp_coordinate)
        # result_list.append(temp_similarity)
        
        # print(candidates_list)
        # print(max(result_list))

        # select the best one and return
        idx = result_list.index(max(result_list))
        return candidates_list[idx]


    






######################### testing ############################
def plot_traj_from_csv():
    x_gt_list = []
    z_gt_list = []
    with open('traj_gt.csv', 'r') as file1:
        csv_reader = csv.reader(file1)
        for row in csv_reader:
            x_gt_list.append(float(row[0]))
            z_gt_list.append(float(row[2]))

    x_pd_list = []
    z_pd_list = []
    with open('traj_pd.csv', 'r') as file2:
        csv_reader = csv.reader(file2)
        for row in csv_reader:
            x_pd_list.append(float(row[0]))
            z_pd_list.append(float(row[2]))

    # fig, axes = plt.subplots(nrows=4, ncols=1)
    plt.figure(figsize=(12, 12))
    plt.plot(x_gt_list, z_gt_list)
    plt.scatter(x_gt_list, z_gt_list, c='blue', s=5)
    plt.plot(x_pd_list, z_pd_list)
    plt.scatter(x_pd_list, z_pd_list, c='red', s=5)

    # bound relation plot
    # for i in range(len(x_gt_list)):
    #     temp_x = [x_gt_list[i], x_pd_list[i]]
    #     temp_z = [z_gt_list[i], z_pd_list[i]]
    #     plt.plot(temp_x, temp_z)

    plt.show()
    # plt.savefig('data_phase1.png')


def plot_phase_dist_from_csv():
    phase_1 = []
    phase_2 = []
    phase_3 = []
    phase_4 = []
    dist_1 = []
    dist_2 = []
    dist_3 = []
    dist_4 = []
    with open('phase_dist.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            phase_1.append(float(row[0]) * WAVE_LENGTH / (4 * np.pi))
            phase_2.append(float(row[1]) * WAVE_LENGTH / (4 * np.pi))
            phase_3.append(float(row[2]) * WAVE_LENGTH / (4 * np.pi))
            phase_4.append(float(row[3]) * WAVE_LENGTH / (4 * np.pi))
            # phase_1.append(float(row[0]))
            # phase_2.append(float(row[1]))
            # phase_3.append(float(row[2]))
            # phase_4.append(float(row[3]))
            dist_1.append(float(row[4]))
            dist_2.append(float(row[5]))
            dist_3.append(float(row[6]))
            dist_4.append(float(row[7]))

    fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(20, 8))
    axes[0].plot(phase_1, label='calibrated unwarpped phase')
    axes[0].plot(dist_1, label='calculated by distance')
    axes[0].set_ylabel('ant1_uw_ph (rad)')

    axes[1].plot(phase_2, label='calibrated unwarpped phase')
    axes[1].plot(dist_2, label='calculated by distance')
    axes[1].set_ylabel('ant2_uw_ph (rad)')

    axes[2].plot(phase_3, label='calibrated unwarpped phase')
    axes[2].plot(dist_3, label='calculated by distance')
    axes[2].set_ylabel('ant3_uw_ph (rad)')

    axes[3].plot(phase_4, label='calibrated unwarpped phase')
    axes[3].plot(dist_4, label='calculated by distance')
    axes[3].set_ylabel('ant4_uw_ph (rad)')
    axes[3].set_xlabel('samples (time)')

    plt.xlabel('samples (time)')
    plt.ylabel('unwrapped phase (rad)')
    plt.legend()
    plt.show()
    # plt.savefig('pure_vertical_tuned.png')





def solve_triangle(ant_right, ant_left, side_left, side_right):
    side_set = abs(ant_right - ant_left) * HALF_SIDE * 2

    # Check if the triangle is valid
    if side_set <= 0 or side_left <= 0 or side_right <= 0:
        return "Invalid triangle: sides must be positive numbers."
    if side_set + side_left <= side_right or side_set + side_right <= side_left or side_left + side_right <= side_set:
        return "Invalid triangle: sum of two sides must be greater than the third side."

    # Calculate angles using the law of cosines
    # angle_top = math.acos((side_left**2 + side_right**2 - side_set**2) / (2 * side_left * side_right))
    # angle_right = math.acos((side_set**2 + side_right**2 - side_left**2) / (2 * side_set * side_right))
    angle_left = math.acos((side_set**2 + side_left**2 - side_right**2) / (2 * side_set * side_left))

    # Calculate the area using the law of sines
    # semiperimeter = (side_set + side_left + side_right) / 2
    # area = math.sqrt(semiperimeter * (semiperimeter - side_set) * (semiperimeter - side_left) * (semiperimeter - side_right))

    # Calculate the coordinate of the tag
    # ant_left_coordinate = np.array([-0.25 * ant_left + 0.625, 0])
    # ant_right_coordinate = np.array([-0.25 * ant_right + 0.625, 0])
    tag_coordinate = np.array([-0.25 * ant_left + 0.625 + side_left * np.cos(angle_left), side_left * np.sin(angle_left)])
    
    ### check ###
    print('-----------------')
    print(math.degrees(angle_left))
    print(side_left)

    return tag_coordinate


def solve_traj(ant_1, ant_2):
    pt_x_list = []
    pt_y_list = []
    with open('phase_dist.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            pt_temp = solve_triangle(ant_1, ant_2, float(row[ant_2 - 1]) * WAVE_LENGTH / (4 * np.pi), float(row[ant_1 - 1]) * WAVE_LENGTH / (4 * np.pi))
            pt_x_list.append(pt_temp[0])
            pt_y_list.append(pt_temp[1])
    
    plt.figure(figsize=(12, 12))
    plt.plot(pt_x_list, pt_y_list)
    plt.scatter(pt_x_list, pt_y_list, c='red', s=5)


    # plt.show()
    plt.savefig('3-4.png')




if __name__ == '__main__':
    
    # try:
    #     rospy.init_node('tag_positioning', anonymous = True)
    #     rate = rospy.Rate(100) # f=100, T=0.001s

    #     # option 1
    #     while not rospy.is_shutdown():
    #         # rospy.Subscriber("/rfid_message", rfid_msg, lambda msg: rfid_callback(msg, phasor_unwrapped))
            
    #         rfid_subscriber = RFID_Subscriber()

    #         rate.sleep() #
    #         rospy.spin() #

    #     # option 2
    #     # rfid_subscriber = RFID_Subscriber()
    #     # rospy.spin()

    # except rospy.ROSInterruptException:
    #     pass



    ########### only for testing ##########
    # plot_traj_from_csv()
    # plot_phase_dist_from_csv()
    solve_traj(3, 4)


























# extra work: putting the rosrun into roslaunch, and embedded the IP address
# coding on real-time variables illustration for convenient debug
# file type, csv more useful, txt not useful

# try ellipse or hyperbolic; try spliting into lateral and horizontal direction
# different backbone, take difference first, then based on this make estimation
# if follow the former method, how to update correctly?