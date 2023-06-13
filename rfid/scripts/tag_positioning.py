import cv2
import rospy
import random
import apriltag
import numpy as np
import pyrealsense2 as rs
from cmath import exp
from rfid.msg import rfid_msg

import csv
import matplotlib.pyplot as plt


################### modified by Rong Zhiyi ####################

################## embedded in a config file ##################
TAG_ID = 'E280-1160-6000-0209-F811-48C3'
PHASE_LOAD_NUM = 4
TAG_SIZE = 0.16
# antenna params
NUM_ANTENNA = 4
WAVE_LENGTH = 0.232924707 # tuned
# WAVE_LENGTH = 0.162924707 * 2 # provided
# camera params
F_X, F_Y = 649.376, 649.376 # focal_length
C_X, C_Y = 648.137, 353.517 # principal_point
# structural params
X_OFFSET = 0.032
Y_OFFSET = 0.085
Z_OFFSET = 0.06
HALF_SIDE = 0.125
# candidates params
RADIUS = 0.02 # metre
NUM_CANDIDATES = 50
####################### saving data ###########################
# file1 = open("phase_unwrap_1.txt", "a+")
# file2 = open("phase_unwrap_2.txt", "a+")
# file3 = open("phase_unwrap_3.txt", "a+")
# file4 = open("phase_unwrap_4.txt", "a+")
# file5 = open("dist_camera_1.txt", "a+")
# file6 = open("dist_camera_2.txt", "a+")
# file7 = open("dist_camera_3.txt", "a+")
# file8 = open("dist_camera_4.txt", "a+")

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
        self.camera_init()
        # self.tag_camera_position = None
        ### callback_1, multi-aperture version
        # self.phasor_unwrapped = []
        # self.phase_old = None # None, 0
        # self.phase_current = None # None, 0
        ### callback_2, parallel version
        self.phasor_unwrapped = np.array([None] * NUM_ANTENNA)
        self.phasor_old = np.array([None] * NUM_ANTENNA)
        self.phasor_current = np.array([None] * NUM_ANTENNA)
        ### 
        # self.location_current = self.tag_camera_position # refer to the camera frame ##### 1
        ### ROS
        self.subscriber = rospy.Subscriber('/rfid_message', rfid_msg, self.callback_2) # choose callback


    def camera_init(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) # depth
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) # color
        self.pipeline.start(self.config)
        self.tag_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
        self.tag_detect() # initialize tag_camera_position
        self.location_current = np.array([self.tag_camera_position[0], self.tag_camera_position[1], self.tag_camera_position[2]])
        


    def tag_detect(self):
        frames = self.pipeline.wait_for_frames()
        # depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # depth_image = np.asanyarray(depth_frame.get_data()) # numpy.ndarray
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        # show images, only for illustration
        # cv2.imshow('RealSense', color_image)
        # key = cv2.waitKey(1)

        tags_list = self.tag_detector.detect(gray)
        for tags in tags_list:
            # add selectioin on tags (environment disturbance exists)
            if tags.tag_id == 0:
                pose, _, _ = self.tag_detector.detection_pose(tags, camera_params=(F_X, F_Y, C_X, C_Y), tag_size=TAG_SIZE)
                self.tag_camera_position = pose[:3, 3]
                # rotation = pose[:3, :3]


    def tag_antenna_position(self, tag_camera_position, antenna_id):
        # position = self.tag_camera_position # if written like this, tag_camera_position will be replaced by the calculated position
        position = np.array([tag_camera_position[0], tag_camera_position[1], tag_camera_position[2]])
        position[0] = position[0] + (2 * antenna_id - 5) * HALF_SIDE - X_OFFSET # 1:-3 2:-1 3:+1 4:+3 
        position[1] = position[1] - Y_OFFSET - HALF_SIDE
        position[2] = position[2] + Z_OFFSET
        return position


    def camera_shutdown(self):
        self.pipeline.stop()


    # multiple apertures version (single antenna)
    def callback_1(self, msg):
        ########################################
        ############# message type #############
        # epc: "E280-1160-6000-0209-F811-48C3"
        # time: 1684307980026460
        # idx: 3
        # mode: 2
        # ant: 4
        # phase: 333.28125
        # rssi: -59.0
        ########################################
        ########################################
        # considering the single antenna case
        if msg.epc == TAG_ID and msg.ant == 1:
            if len(self.phasor_unwrapped) == 0:
                self.phasor_unwrapped.append(msg.phase / 180 * np.pi)
                self.phase_old = msg.phase / 180 * np.pi
            else:
                self.phase_current = msg.phase / 180 * np.pi
                if self.phase_current - self.phase_old > np.pi:
                    self.phasor_unwrapped.append(self.phasor_unwrapped[-1] + self.phase_current - self.phase_old - 2 * np.pi)
                elif self.phase_old - self.phase_current > np.pi:
                    self.phasor_unwrapped.append(self.phasor_unwrapped[-1] + self.phase_current - self.phase_old + 2 * np.pi)
                else:
                    self.phasor_unwrapped.append(self.phasor_unwrapped[-1] + self.phase_current - self.phase_old)

                if len(self.phasor_unwrapped) > PHASE_LOAD_NUM:
                    del self.phasor_unwrapped[0] # phasor_unwrapped.pop(0)
                self.phase_old = self.phase_current

                # recording phase in .txt file
                # file2.write(str(self.phasor_unwrapped[-1]) + "\n")
                # file1.write(str(self.phase_current) + "\n")
                
            print('-------------------------------')
            print(self.phasor_unwrapped)
            # print(self.phasor_normalized(self.phasor_unwrapped))

            self.tag_detect()

            print(self.tag_camera_position)
            print(self.tag_antenna_position(self.tag_camera_position, msg.ant))
            print(np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant)))

            # file3.write(str(np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant))) + "\n") # true euclidean distance
            # file3.write(str(self.tag_camera_position[2]) + "\n") # distance along z axis



    # multiple antennas (parallel)
    def callback_2(self, msg):

        if msg.epc == TAG_ID:

            if self.phasor_unwrapped[msg.ant - 1] == None:
            
                # original
                # self.phasor_unwrapped[msg.ant - 1] = msg.phase / 180 * np.pi
                # self.phasor_old[msg.ant - 1] = msg.phase / 180 * np.pi
                
                # modified by introducing initial calibration
                self.phasor_unwrapped[msg.ant - 1] = np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant)) * 4 * np.pi / WAVE_LENGTH

                # self.phasor_unwrapped[msg.ant - 1] = -msg.phase / 180 * np.pi
                self.phasor_old[msg.ant - 1] = msg.phase / 180 * np.pi
                

                # modified_unwrapped_phase.append(-float(line2) + float(data2[0]) + float(data3[0]) * 4 * np.pi / lmd)
                # np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant))

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
                    self.tag_detect() # messages for 4 antennas can be seen simultaneous, where detection only need to be done once
                    
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

                    

                    # updating the current cordinate with the most possible candidate
                    predicted_coordinate = self.candidates_generator()
                    self.location_current = predicted_coordinate

                    traj_gt_writer.writerow([self.tag_camera_position[0], self.tag_camera_position[1], self.tag_camera_position[2]])
                    traj_pd_writer.writerow([predicted_coordinate[0], predicted_coordinate[1], predicted_coordinate[2]])



            # every time processing ant4 and when there is no None in phasor_unwarpped, do the matching




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

        # if random, no need to include itself
        # temp_coordinate = np.array((self.location_current[0], self.location_current[1], self.location_current[2]))
        # temp_phasor = np.array([np.linalg.norm(self.tag_antenna_position(temp_coordinate, 1)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 2)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 3)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 4)) * 4 * np.pi / WAVE_LENGTH])
        # temp_similarity = self.cosine_similarity(self.phasor_normalized(self.phasor_unwrapped), self.phasor_normalized(temp_phasor))
        # candidates_list.append(temp_phasor)
        # result_list.append(temp_similarity)
        
        # print(candidates_list)
        # print(result_list)

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

    # for i in range(len(x_gt_list)):
    #     temp_x = [x_gt_list[i], x_pd_list[i]]
    #     temp_z = [z_gt_list[i], z_pd_list[i]]
    #     plt.plot(temp_x, temp_z)

    plt.show()
    # plt.savefig('data_phase1.png')

    # plt.xlabel('X of tf_translation (m)')
    # plt.title('data5')
    # plt.axis('equal')



# offline unwrap method
# def data_unwrap(array, scale): # scale=1 is chosen as the best constant
#     current_layer = 0
#     unwrap_array = []
#     for i in range(len(array)-1):
#         unwrap_array.append(array[i] + current_layer*2*np.pi)
#         if array[i] - array[i+1] > scale*np.pi:
#             current_layer = current_layer + 1
#         elif array[i+1] - array[i] > scale*np.pi:
#             current_layer = current_layer - 1
#     return unwrap_array




def plot_bi_compare():
    file = open("phase_unwrap.txt", "r")
    dist = file.readlines() # class str
    phase_unwrap = []
    for line in dist:
        phase_unwrap.append(float(line))

    file2 = open("dist_camera.txt", "r")
    dist2 = file2.readlines() # class str
    distance = []
    for line2 in dist2:
        distance.append(float(line2))

    plt.plot(phase_unwrap)
    plt.plot(distance)
    # plt.plot(np.unwrap(phase_unwrap))
    # plt.xlabel('X of tf_translation (m)')
    plt.ylabel('Unwrapped Phase (rad)')
    # plt.title('data5')
    # plt.axis('equal')
    plt.show()
    # plt.savefig('wrapped_phase.png')



def plot_multi():
    file1 = open("phase_unwrap_1.txt", "r")
    file2 = open("phase_unwrap_2.txt", "r")
    file3 = open("phase_unwrap_3.txt", "r")
    file4 = open("phase_unwrap_4.txt", "r")
    file5 = open("dist_camera_1.txt", "r")
    file6 = open("dist_camera_2.txt", "r")
    file7 = open("dist_camera_3.txt", "r")
    file8 = open("dist_camera_4.txt", "r")

    data1 = file1.readlines()
    data2 = file2.readlines()
    data3 = file3.readlines()
    data4 = file4.readlines()
    data5 = file5.readlines()
    data6 = file6.readlines()
    data7 = file7.readlines()
    data8 = file8.readlines()
    
    list1 = []
    list2 = []
    list3 = []
    list4 = []
    list5 = []
    list6 = []
    list7 = []
    list8 = []

    for line1 in data1:
        list1.append(float(line1))
    for line2 in data2:
        list2.append(float(line2))
    for line3 in data3:
        list3.append(float(line3))
    for line4 in data4:
        list4.append(float(line4))
    for line5 in data5:
        list5.append(float(line5))
    for line6 in data6:
        list6.append(float(line6))
    for line7 in data7:
        list7.append(float(line7))
    for line8 in data8:
        list8.append(float(line8))

    # for line1 in data1:
    #     d_list1.append(float(line1))
    # for line2 in data2:
    #     # phase2dist.append(-float(line2) * lmd / (4 * np.pi) + float(data3[0]) + float(data2[0]) * lmd / (4 * np.pi)) # regularize with same initial
    #     d_list2.append(float(line2))
    #     modified_unwrapped_phase.append(-float(line2) + float(data2[0]) + float(data3[0]) * 4 * np.pi / lmd) # 
    # for line3 in data3:
    #     d_list3.append(float(line3))
    #     dist2phase.append(float(line3) * 4 * np.pi / lmd) # distance should not be negative
    
    fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(20, 8))
    axes[0].plot(list1, label='calibrated unwarpped phase')
    axes[0].plot(list5, label='calculated by distance')
    axes[0].set_ylabel('ant1_uw_ph (rad)')

    axes[1].plot(list2, label='calibrated unwarpped phase')
    axes[1].plot(list6, label='calculated by distance')
    axes[1].set_ylabel('ant2_uw_ph (rad)')

    axes[2].plot(list3, label='calibrated unwarpped phase')
    axes[2].plot(list7, label='calculated by distance')
    axes[2].set_ylabel('ant3_uw_ph (rad)')

    axes[3].plot(list4, label='calibrated unwarpped phase')
    axes[3].plot(list8, label='calculated by distance')
    axes[3].set_ylabel('ant4_uw_ph (rad)')
    axes[3].set_xlabel('samples (time)')

    # plt.plot(list1, label='antenna1')
    # plt.plot(list2, label='antenna2')
    # plt.plot(list3, label='antenna3')
    # plt.plot(list4, label='antenna4')
    plt.xlabel('samples (time)')
    plt.ylabel('unwrapped phase (rad)')


    plt.legend()

    plt.show()
    # plt.savefig('1_0.23_wrapped.png')







if __name__ == '__main__':
    
    # try:
    #     rospy.init_node('tag_positioning', anonymous = True)
    #     rate = rospy.Rate(100)

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
    plot_traj_from_csv()
    # plot_bi_compare()
    # plot_multi()



# extra work: putting the rosrun into roslaunch, and embedded the IP address
# coding on real-time variables illustration for convenient debug
# file type, csv more useful, txt not useful

# try ellipse or hyperbolic; try spliting into lateral and horizontal direction
# different backbone, take difference first, then based on this make estimation
# if follow the former method, how to update correctly?