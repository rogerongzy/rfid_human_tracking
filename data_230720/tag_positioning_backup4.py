import cv2
import sys
import csv
import math
import rospy
import random
import apriltag
import numpy as np
import pyrealsense2 as rs
import matplotlib.pyplot as plt
from cmath import exp
from rfid.msg import rfid_msg
from geometry_msgs.msg import Twist



################### modified by Rong Zhiyi ####################

################## embedded in a config file ##################
# TAG_ID = 'E280-1160-6000-0209-F811-48C3' # white, apriltag
TAG_ID = 'E280-1160-6000-0209-F811-5C03' # brown, aruco
PHASE_LOAD_NUM = 4
TAG_SIZE = 0.16
# antenna params
NUM_ANTENNA = 4
# WAVE_LENGTH = 0.164624707 * 2 # tuned
WAVE_LENGTH = 0.162924707 * 2 # provided
# camera params
F_X, F_Y = 649.376, 649.376 # focal_length
C_X, C_Y = 648.137, 353.517 # principal_point
# structural params
X_OFFSET = 0.032
Y_OFFSET = 0.085
Z_OFFSET = 0.06
HALF_SIDE = 0.125
# candidates generator params
RADIUS = 0.05 # metre
NUM_CANDIDATES = 100
# starting point used for manual-grid-calibration
# location_initial = np.array([-0.5, 0, 1])
# P controller params
KP_linear = 0.08
KP_angular = 0.008
####################### saving data ###########################
phase_dist_file = open('phase_dist.csv', 'a+')
phase_dist_writer = csv.writer(phase_dist_file)
# traj_gt_file = open('traj_gt.csv', 'a+')
# traj_gt_writer = csv.writer(traj_gt_file)
traj_pd_file = open('traj_pd.csv', 'a+')
traj_pd_writer = csv.writer(traj_pd_file)





###############################################################
# considering the single antenna first
# phase_loader consists of 4 consecutive phase (can be interrupted)

class RFID_Subscriber:
    def __init__(self):
        ### camera ###
        # self.camera_init()

        ### callback_1, multi-aperture version
        # self.phasor_unwrapped = []
        # self.phase_old = None # None, 0
        # self.phase_current = None # None, 0

        ### callback_2, parallel version
        self.phasor_unwrapped = np.array([None] * NUM_ANTENNA)
        self.phasor_old = np.array([None] * NUM_ANTENNA)
        self.phasor_current = np.array([None] * NUM_ANTENNA)
        self.rssi_list = np.array([None] * NUM_ANTENNA)
        
        ### ROS ###
        self.subscriber = rospy.Subscriber('/rfid_message', rfid_msg, self.callback_2) # choose callback 1 or 2
        self.publisher = rospy.Publisher('/dynamixel_workbench/cmd_vel', Twist, queue_size=10)
        self.action_timestamp = None # used to adjust the processing period
        
        ### calibration ###
        # self.location_current = location_initial
        
        ### control ###
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0

        ### state machine ###
        self.state_status = 2 # 0 for initial

        # used for trajectory localization
        self.flag = True
        self.begin_time = None
        self.begin_phasor = np.array([None] * NUM_ANTENNA)



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

            # Go straight (or some other searching)
            # until one of the rssi is smaller than abs(-60)
            # adjust direction to make it point towards

            if self.state_status == 0:
                print('-----state 0-----') # go straight for large enough rssi

                self.twist_msg.linear.x = 0.04
                self.twist_msg.angular.z = 0.0
                
                if msg.ant == 4:
                    self.publisher.publish(self.twist_msg)

                self.rssi_list[msg.ant - 1] = msg.rssi

                if (self.rssi_list[0] != None and abs(self.rssi_list[0]) < 50) or (self.rssi_list[3] != None and abs(self.rssi_list[3]) < 50):
                    self.twist_msg.linear.x = 0.0
                    self.twist_msg.angular.z = 0.0
                    self.publisher.publish(self.twist_msg)
                    self.state_status = 1
                
                
            elif self.state_status == 1:
                print('-----state 1-----') # turn until rssi from ant1 and ant4 is approximate

                self.rssi_list[msg.ant - 1] = msg.rssi
                print(self.rssi_list)

                if self.rssi_list[0] != None and self.rssi_list[3] != None:
                    if abs(self.rssi_list[0] - self.rssi_list[3]) < 0.5:
                        self.twist_msg.linear.x = 0.0
                        self.twist_msg.angular.z = 0.0
                        self.state_status = 2
                    elif self.rssi_list[0] < self.rssi_list[3]:
                        self.twist_msg.angular.z = 0.1
                        # print('turn left')
                    elif self.rssi_list[0] > self.rssi_list[3]:
                        self.twist_msg.angular.z = -0.1
                        # print('turn right')
                
                if msg.ant == 4:
                    self.publisher.publish(self.twist_msg)

                
                
            elif self.state_status == 2:
                print('-----state 2-----') # to go for a known/preset distance
                
                if self.flag:
                    self.begin_time = msg.time
                    self.flag = False

                # phase unwrap
                if self.phasor_unwrapped[msg.ant - 1] == None:
                    self.begin_phasor[msg.ant - 1] = msg.phase / 180 * np.pi ## used to record the very initial phase
                    self.phasor_unwrapped[msg.ant - 1] = msg.phase / 180 * np.pi # no calibration
                    self.phasor_old[msg.ant - 1] = msg.phase / 180 * np.pi
                    self.action_timestamp = msg.time
                else:                   
                    self.phasor_current[msg.ant - 1] = msg.phase / 180 * np.pi
                    df_ph = self.phasor_current[msg.ant - 1] - self.phasor_old[msg.ant - 1]
                    df_upper = df_ph + 2 * np.pi
                    df_lower = df_ph - 2 * np.pi
                    df_list = [df_lower, df_ph, df_upper]
                    df_abs_list = [abs(df_lower), abs(df_ph), abs(df_upper)]
                    idx = df_abs_list.index(min(df_abs_list))
                    self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] - df_list[idx]
                    self.phasor_old[msg.ant - 1] = self.phasor_current[msg.ant - 1]


                self.twist_msg.linear.x = 0.06 # 0.02 for 27s
                self.twist_msg.angular.z = 0.0
                if msg.ant == 4:
                    self.publisher.publish(self.twist_msg)
                    # record the phase data
                    # phase_dist_writer.writerow([self.phasor_unwrapped[0], self.phasor_unwrapped[1], self.phasor_unwrapped[2], self.phasor_unwrapped[3], 0, 0, 0, 0])

                # print(msg.time)
                # print(msg.time - self.begin_time)
                if msg.time - self.begin_time > 17.8 * 1000000: # 54s is preset to approach 1m, 27 for 0.5m, 8.9 & 0.06 for 0.5m
                    self.twist_msg.linear.x = 0.0
                    self.twist_msg.angular.z = 0.0
                    self.publisher.publish(self.twist_msg)
                    self.state_status = 9

                    # solve the hyperbolas intersaction
                    var_ph_ant1 = (self.begin_phasor[0] - self.phasor_unwrapped[0]) * WAVE_LENGTH / (4 * np.pi) # in meters
                    var_ph_ant4 = (self.begin_phasor[3] - self.phasor_unwrapped[3]) * WAVE_LENGTH / (4 * np.pi)
                    dist = 1 # the preset moving-forward distance

                    # iterator from -0.375 to 0.375, step = 0.001
                    x = -0.376
                    val_result = []
                    for _ in range(751):
                        x += 0.001
                        val1 = ((x-0.375)**2) * ((var_ph_ant1/2)**2) / (((dist/2)**2) - ((var_ph_ant1/2)**2)) + ((var_ph_ant1/2)**2)
                        val4 = ((x+0.375)**2) * ((var_ph_ant4/2)**2) / (((dist/2)**2) - ((var_ph_ant4/2)**2)) + ((var_ph_ant4/2)**2)
                        val_temp = abs(val1 - val4)
                        val_result.append(val_temp)
                    
                    idx_solution = val_result.index(min(val_result))
                    x_solution = -0.376 + idx_solution * 0.001
                    y_solution = np.sqrt(((x_solution-0.375)**2) * ((var_ph_ant1/2)**2) / (((dist/2)**2) - ((var_ph_ant1/2)**2)) + ((var_ph_ant1/2)**2)) - (dist/2)
                    print(val_result)

                    print('ant1: ', var_ph_ant1)
                    print('ant4: ', var_ph_ant4)
                    print('idx: ', idx_solution)
                    print('x: ', x_solution)
                    print('y: ', y_solution)

                    self.location_current = np.array([x_solution, 0, y_solution])
                    self.phasor_unwrapped = np.array([None] * NUM_ANTENNA) # renew the data for following calibration


            elif self.state_status == 9:
                print('-----buffer-----')


            elif self.state_status == 3:
                print('-----state 3-----')

                if self.phasor_unwrapped[msg.ant - 1] == None:
                
                    # original
                    # self.phasor_unwrapped[msg.ant - 1] = msg.phase / 180 * np.pi
                    # self.phasor_old[msg.ant - 1] = msg.phase / 180 * np.pi

                    # modified by introducing initial calibration
                    # self.phasor_unwrapped[msg.ant - 1] = np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant)) * 4 * np.pi / WAVE_LENGTH # calibration using camera rgb stream
                    self.phasor_unwrapped[msg.ant - 1] =  np.linalg.norm(self.tag_antenna_position(self.location_current, msg.ant)) * 4 * np.pi / WAVE_LENGTH # calibration using manual grid
                    # self.phasor_unwrapped[msg.ant - 1] = msg.phase / 180 * np.pi # no calibration

                    self.phasor_old[msg.ant - 1] = msg.phase / 180 * np.pi

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

                    ## unwrapping method 1, detect variation within 2*pi
                    # if self.phasor_current[msg.ant - 1] - self.phasor_old[msg.ant - 1] > np.pi:
                    #     self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] - self.phasor_current[msg.ant - 1] + self.phasor_old[msg.ant - 1] + 2 * np.pi
                    # elif self.phasor_old[msg.ant - 1] - self.phasor_current[msg.ant - 1] > np.pi:
                    #     self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] - self.phasor_current[msg.ant - 1] + self.phasor_old[msg.ant - 1] - 2 * np.pi
                    # else:
                    #     self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] - self.phasor_current[msg.ant - 1] + self.phasor_old[msg.ant - 1]
                    #####

                    ## unwrapped method 2, compare difference
                    df_ph = self.phasor_current[msg.ant - 1] - self.phasor_old[msg.ant - 1]
                    df_upper = df_ph + 2 * np.pi
                    df_lower = df_ph - 2 * np.pi
                    df_list = [df_lower, df_ph, df_upper]
                    df_abs_list = [abs(df_lower), abs(df_ph), abs(df_upper)]
                    idx = df_abs_list.index(min(df_abs_list))
                    self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] - df_list[idx]
                    #####

                    self.phasor_old[msg.ant - 1] = self.phasor_current[msg.ant - 1] # original place


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



                            ########## lei-particle filter ##########
                            ### updating the current cordinate with the most possible candidate (complete process)
                            # predicted_coordinate = self.candidates_generator()
                            # self.location_current = predicted_coordinate

                            ### writer for trajectory, both ground truth and predicted ###
                            # traj_gt_writer.writerow([self.tag_camera_position[0], self.tag_camera_position[1], self.tag_camera_position[2]]) # camera no more 
                            # traj_pd_writer.writerow([predicted_coordinate[0], predicted_coordinate[1], predicted_coordinate[2]])

                            ### writer for phase and distance ###
                            # phase_dist_writer.writerow([self.phasor_unwrapped[0], self.phasor_unwrapped[1], self.phasor_unwrapped[2], self.phasor_unwrapped[3], np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, 1)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, 2)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, 3)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, 4)) * 4 * np.pi / WAVE_LENGTH])
                            # phase_dist_writer.writerow([self.phasor_unwrapped[0], self.phasor_unwrapped[1], self.phasor_unwrapped[2], self.phasor_unwrapped[3], 0, 0, 0, 0])



                            ########## geometry triangle solving ##########
                            ### solving triangle using ant 1 and 4, in 2D version x-y plane
                            tag_antref_position = solve_triangle(1, 4, self.phasor_unwrapped[3] * WAVE_LENGTH / (4 * np.pi), self.phasor_unwrapped[0] * WAVE_LENGTH / (4 * np.pi))
                            # print(tag_antref_position) # record the trajectory using 2D coordinate, only used in static robot
                            # traj_pd_writer.writerow([tag_antref_position[0], tag_antref_position[1], 0])

                            r, theta = cartesian_to_polar(tag_antref_position[0], tag_antref_position[1]) # in polar coordinate

                            ### update velocity and publish ##++##
                            if r > 1.1:
                                self.twist_msg.linear.x = KP_linear * (r - 1.1) + 0.05
                            elif r < 0.9:
                                self.twist_msg.linear.x = 0.0

                            self.twist_msg.angular.z = KP_angular * (theta - 90) # oscillation exists

                            self.publisher.publish(self.twist_msg)
                        

                    







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





def solve_triangle(ant_right, ant_left, side_left, side_right):
    side_set = abs(ant_right - ant_left) * HALF_SIDE * 2
    

    # Check if the triangle is valid
    if side_set <= 0 or side_left <= 0 or side_right <= 0:
        return "Invalid triangle: sides must be positive numbers."
    if side_set + side_left <= side_right or side_set + side_right <= side_left or side_left + side_right <= side_set:
        print(side_set)
        print(side_left)
        print(side_right)
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
    # print('-----------------')
    # print(math.degrees(angle_left))
    # print(side_left)

    return tag_coordinate


# def solve_traj(ant_1, ant_2):
#     pt_x_list = []
#     pt_y_list = []
#     with open('phase_dist.csv', 'r') as file:
#         csv_reader = csv.reader(file)
#         for row in csv_reader:
#             pt_temp = solve_triangle(ant_1, ant_2, float(row[ant_2 - 1]) * WAVE_LENGTH / (4 * np.pi), float(row[ant_1 - 1]) * WAVE_LENGTH / (4 * np.pi))
#             pt_x_list.append(pt_temp[0])
#             pt_y_list.append(pt_temp[1])
    
#     plt.figure(figsize=(12, 12))
#     plt.plot(pt_x_list, pt_y_list)
#     plt.scatter(pt_x_list, pt_y_list, c='red', s=5)
#     plt.show()
#     # plt.savefig('3-4.png')



def cartesian_to_polar(x, y):
    # Calculate the radius (distance from origin)
    radius = math.sqrt(x**2 + y**2)

    # Calculate the angle (in radians)
    angle = math.atan2(y, x)

    # Convert the angle from radians to degrees
    angle_degrees = math.degrees(angle)

    # Return the radius and angle in polar coordinates
    return radius, angle_degrees



######################### evaluation ############################
def plot_traj_from_csv():
    # x_gt_list = []
    # z_gt_list = []
    # with open('traj_gt.csv', 'r') as file1:
    #     csv_reader = csv.reader(file1)
    #     for row in csv_reader:
    #         x_gt_list.append(float(row[0]))
    #         z_gt_list.append(float(row[2]))

    x_pd_list = []
    z_pd_list = []
    with open('traj_pd.csv', 'r') as file2:
        csv_reader = csv.reader(file2)
        for row in csv_reader:
            x_pd_list.append(float(row[0]))
            z_pd_list.append(float(row[1]))

    # fig, axes = plt.subplots(nrows=4, ncols=1)
    plt.figure(figsize=(12, 12))
    # plt.plot(x_gt_list, z_gt_list)
    # plt.scatter(x_gt_list, z_gt_list, c='blue', s=5)
    plt.plot(x_pd_list, z_pd_list, label='est_traj')
    plt.scatter(x_pd_list, z_pd_list, c='red', s=5)
    plt.plot([-0.5, -0.5], [1, 4], c='red', label='gt_traj')
    plt.legend()

    # bound relation plot
    # for i in range(len(x_gt_list)):
    #     temp_x = [x_gt_list[i], x_pd_list[i]]
    #     temp_z = [z_gt_list[i], z_pd_list[i]]
    #     plt.plot(temp_x, temp_z)

    # plt.show()
    plt.savefig('estm_traj.png')


def plot_phase_dist_from_csv():
    phase_1 = []
    phase_2 = []
    phase_3 = []
    phase_4 = []
    # dist_1 = []
    # dist_2 = []
    # dist_3 = []
    # dist_4 = []
    df_phase_1 = []
    df_phase_2 = []
    df_phase_3 = []
    df_phase_4 = []
    df_criteria = []
    df_status = False
    with open('phase_dist.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            # phase_1.append(float(row[0]) * WAVE_LENGTH / (4 * np.pi)) # metre unit
            # phase_2.append(float(row[1]) * WAVE_LENGTH / (4 * np.pi))
            # phase_3.append(float(row[2]) * WAVE_LENGTH / (4 * np.pi))
            # phase_4.append(float(row[3]) * WAVE_LENGTH / (4 * np.pi))
            phase_1.append(float(row[0])) # rad unit
            phase_2.append(float(row[1]))
            phase_3.append(float(row[2]))
            phase_4.append(float(row[3]))
            # dist_1.append(float(row[4]))
            # dist_2.append(float(row[5]))
            # dist_3.append(float(row[6]))
            # dist_4.append(float(row[7]))
            if df_status:
                df_phase_1.append(phase_1[-1] - phase_1[-2])
                df_phase_2.append(phase_2[-1] - phase_2[-2])
                df_phase_3.append(phase_3[-1] - phase_3[-2])
                df_phase_4.append(phase_4[-1] - phase_4[-2])
                df_criteria.append(0)
            else:
                df_status = True


    # fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(20, 8))
    # axes[0].plot(phase_1, label='calibrated unwarpped phase')
    # axes[0].plot(dist_1, label='calculated by distance')
    # axes[0].set_ylabel('ant1_uw_ph (rad)')

    # axes[1].plot(phase_2, label='calibrated unwarpped phase')
    # axes[1].plot(dist_2, label='calculated by distance')
    # axes[1].set_ylabel('ant2_uw_ph (rad)')

    # axes[2].plot(phase_3, label='calibrated unwarpped phase')
    # axes[2].plot(dist_3, label='calculated by distance')
    # axes[2].set_ylabel('ant3_uw_ph (rad)')

    # axes[3].plot(phase_4, label='calibrated unwarpped phase')
    # axes[3].plot(dist_4, label='calculated by distance')
    # axes[3].set_ylabel('ant4_uw_ph (rad)')
    # axes[3].set_xlabel('samples (time)')
    
    plt.figure(figsize=(12, 12))
    plt.plot(phase_1, label='ant1')
    plt.plot(phase_2, label='ant2')
    plt.plot(phase_3, label='ant3')
    plt.plot(phase_4, label='ant4')

    plt.xlabel('samples (time)')
    plt.ylabel('unwrapped phase (rad)')
    plt.legend()
    # plt.show()
    plt.savefig('phase.png')

    plt.figure(figsize=(20, 8))
    plt.plot(df_phase_1, label='ant1')
    plt.plot(df_phase_2, label='ant2')
    plt.plot(df_phase_3, label='ant3')
    plt.plot(df_phase_4, label='ant4')
    plt.plot(df_criteria, label='0 criteria')

    plt.xlabel('samples (time)')
    plt.ylabel('unwrapped phase difference (rad)')
    plt.legend()
    # plt.show()
    plt.savefig('df_phase.png')


def plot_phase_singletest():
    phase_1 = []
    cri_start = []
    cri_end = []


    with open('phase_dist.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            phase_1.append(float(row[0]) * WAVE_LENGTH / (4 * np.pi)) # metre unit 
            cri_start.append(1)
            cri_end.append(4)
            # phase_1.append(float(row[0])) # rad unit

    plt.figure(figsize=(12, 12))
    plt.plot(phase_1, label='ant1')
    plt.plot(cri_start, label='1')
    plt.plot(cri_end, label='4')

    plt.xlabel('samples (time)')
    plt.ylabel('unwrapped phase (rad)')
    plt.legend()
    plt.show()
    # plt.savefig('phase.png')

    # plt.figure(figsize=(20, 8))
    # plt.plot(df_phase_1, label='ant1')


    # plt.xlabel('samples (time)')
    # plt.ylabel('unwrapped phase difference (rad)')
    # plt.legend()
    # # plt.show()
    # plt.savefig('df_phase.png')





def main_run():
    try:
        rospy.init_node('tag_positioning', anonymous = True)
        rate = rospy.Rate(100) # f=100, T=0.001s

        # option 1
        while not rospy.is_shutdown():
            # rospy.Subscriber("/rfid_message", rfid_msg, lambda msg: rfid_callback(msg, phasor_unwrapped))
            
            rfid_subscriber = RFID_Subscriber()

            rate.sleep() #
            rospy.spin() #

        # option 2
        # rfid_subscriber = RFID_Subscriber()
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    if sys.argv[1] == '1':
        main_run()
    elif sys.argv[1] == '2':
        plot_traj_from_csv()
    # elif sys.argv[1] == '3':
        plot_phase_dist_from_csv()
    elif sys.argv[1] == '3':
        plot_phase_singletest() 




























# extra work: putting the rosrun into roslaunch, and embedded the IP address
# coding on real-time variables illustration for convenient debug
# file type, csv more useful, txt not useful

# try ellipse or hyperbolic; try spliting into lateral and horizontal direction
# different backbone, take difference first, then based on this make estimation
# if follow the former method, how to update correctly?