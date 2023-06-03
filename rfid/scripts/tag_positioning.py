import cv2
import rospy
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
WAVE_LENGTH = 0.232924707 # 0.162924707 * 2 provided
# camera params
F_X, F_Y = 649.376, 649.376 # focal_length
C_X, C_Y = 648.137, 353.517 # principal_point
# structural params
X_OFFSET = 0.032
Y_OFFSET = 0.085
Z_OFFSET = 0.06
HALF_SIDE = 0.125
# candidates params
RADIUS = 0.2 # metre
NUM_CIRCULAR_SPLIT = 8
###############################################################
# file1 = open("phase_wrap.txt", "a+")
# file2 = open("phase_unwrap.txt", "a+")
# file3 = open("dist_camera.txt", "a+")


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
                self.location_current = self.tag_camera_position ##### 2


    def tag_antenna_position(self, tag_camera_position, antenna_id):
        # position = self.tag_camera_position # if written like this, tag_camera_position will be replaced by the calculated position
        position = np.array((tag_camera_position[0], tag_camera_position[1], tag_camera_position[2]))
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
                
                # modified
                # initial calibration
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
                
                # modified
                self.phasor_current[msg.ant - 1] = msg.phase / 180 * np.pi

                if self.phasor_current[msg.ant - 1] - self.phasor_old[msg.ant - 1] > np.pi:
                    self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] - self.phasor_current[msg.ant - 1] + self.phasor_old[msg.ant - 1] + 2 * np.pi
                elif self.phasor_old[msg.ant - 1] - self.phasor_current[msg.ant - 1] > np.pi:
                    self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] - self.phasor_current[msg.ant - 1] + self.phasor_old[msg.ant - 1] - 2 * np.pi
                else:
                    self.phasor_unwrapped[msg.ant - 1] = self.phasor_unwrapped[msg.ant - 1] - self.phasor_current[msg.ant - 1] + self.phasor_old[msg.ant - 1]
            
                self.phasor_old[msg.ant - 1] = self.phasor_current[msg.ant - 1]


                if msg.ant == 4:
                    self.tag_detect() # messages for 4 antennas can be seen simultaneous, where detection only need to be done once
                    
                    print('-------------------------------')
                    print(self.phasor_unwrapped)

                    # print(self.phasor_normalized(self.phasor_unwrapped))
                    # print(np.linalg.norm(self.phasor_normalized(self.phasor_unwrapped)))
                    # print(self.phasor_normalized(self.phasor_unwrapped).getH())
                    # print(np.linalg.norm(self.phasor_normalized(self.phasor_unwrapped).getH()))
                    # print(self.cosine_similarity(self.phasor_normalized(self.phasor_unwrapped), self.phasor_normalized(self.phasor_unwrapped)))


                    # print(self.location_current)
                    # print(self.tag_camera_position)
                    # print(self.tag_antenna_position(self.tag_camera_position, msg.ant))
                    print(self.candidates_generator())


                    # file2.write(str(self.phasor_unwrapped[msg.ant - 1]) + "\n")
                    # file3.write(str(np.linalg.norm(self.tag_antenna_position(self.tag_camera_position, msg.ant)) * 4 * np.pi / WAVE_LENGTH) + "\n")
            



            # every time processing ant4 and when there is no None in phasor_unwarpped, do the matching




    def phasor_normalized(self, phasor):
        phasor_temp = []
        for i in range(len(phasor)):
            phasor_temp.append(exp(-1j*(phasor[i] - phasor[0])))
        phasor_normalized = np.matrix(phasor_temp)
        return phasor_normalized


    def cosine_similarity(self, complex_vec_1, complex_vec_2):
        num = np.linalg.norm(np.dot(complex_vec_1, complex_vec_2.getH()))
        # num = np.dot(complex_vec_1, complex_vec_2.getH())
        # print(num)
        den = np.linalg.norm(complex_vec_1) * np.linalg.norm(complex_vec_2)
        # print(den)
        return num / den


    def candidates_generator(self):
        # only in 2D top-view plane
        candidates = []
        for i in range(NUM_CIRCULAR_SPLIT):
            # generate a circle
            temp_coordinate = np.array((self.location_current[0] + RADIUS * np.cos(i * 2 * np.pi / NUM_CIRCULAR_SPLIT), self.location_current[1], self.location_current[2] + RADIUS * np.sin(i * 2 * np.pi / NUM_CIRCULAR_SPLIT)))
            # generate a ellipse instead



            temp_phasor = np.array([np.linalg.norm(self.tag_antenna_position(temp_coordinate, 1)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 2)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 3)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 4)) * 4 * np.pi / WAVE_LENGTH])
            
            temp_similarity = self.cosine_similarity(self.phasor_normalized(self.phasor_unwrapped), self.phasor_normalized(temp_phasor))

            candidates.append(temp_phasor)

        # need to include itself?
        temp_coordinate = np.array((self.location_current[0], self.location_current[1], self.location_current[2]))
        temp_phasor = np.array([np.linalg.norm(self.tag_antenna_position(temp_coordinate, 1)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 2)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 3)) * 4 * np.pi / WAVE_LENGTH, np.linalg.norm(self.tag_antenna_position(temp_coordinate, 4)) * 4 * np.pi / WAVE_LENGTH])
        temp_similarity = self.cosine_similarity(self.phasor_normalized(self.phasor_unwrapped), self.phasor_normalized(temp_phasor))
        candidates.append(temp_phasor)

        return candidates


    ########################
    # seems to be sensative along lateral (z) direction, maybe use ellipse instead?

    # when updating, not appriximate directly, do synthesis

    def location_update(self):
        self.location_current = self.location_current











######################### testing ############################
def only_test():
    x = np.matrix(np.arange(4).reshape((1, 4)))
    z = x - 1j * x
    print(type(z))
    # z.getH(), complex conjugate transpose of a matrix
    print(z.getH()) 

    # evaluation criteria
    # |A * C| / |A| * |C|
    # print(exp(1j))



# def read_csv():
#     phase1 = []
#     count = 0
#     # Open the CSV file
#     with open('data.csv', 'r') as file:
#         # Create a CSV reader object
#         csv_reader = csv.reader(file)

#         # Read the header (optional)
#         # header = next(csv_reader)

#         # Process each row in the CSV file
#         for row in csv_reader:
#             if count < 300:
#                 phase1.append(float(row[0]))
#                 count = count + 1

#     fig, axes = plt.subplots(nrows=4, ncols=1)
#     axes[0].plot(phase1)
#     axes[1].plot(data_unwrap(phase1, 1.5))
#     axes[2].plot(data_unwrap(phase1, 1)) # best
#     axes[3].plot(data_unwrap(phase1, 0.25))
#     plt.show()
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
    file1 = open("phase_wrap.txt", "r")
    file2 = open("phase_unwrap.txt", "r")
    file3 = open("dist_camera.txt", "r")
    data1 = file1.readlines()
    data2 = file2.readlines()
    data3 = file3.readlines()
    d_list1 = []
    d_list2 = []
    d_list3 = []

    phase2dist = []
    dist2phase = []
    modified_unwrapped_phase = []

    # lmd = 0.162924707 * 2 # provided
    
    # lmd = 0.162924707 # ant1
    # lmd = 0.162924707 # ant4

    # lmd = 0.202924707 # ant2
    # lmd = 0.202924707 # ant3
    
    lmd = 0.232924707 # unity, 22 or 23

    for line1 in data1:
        d_list1.append(float(line1))
    for line2 in data2:
        # phase2dist.append(-float(line2) * lmd / (4 * np.pi) + float(data3[0]) + float(data2[0]) * lmd / (4 * np.pi)) # regularize with same initial
        d_list2.append(float(line2))
        modified_unwrapped_phase.append(-float(line2) + float(data2[0]) + float(data3[0]) * 4 * np.pi / lmd) # 
    for line3 in data3:
        d_list3.append(float(line3))
        dist2phase.append(float(line3) * 4 * np.pi / lmd) # distance should not be negative
    
    fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(20, 8))
    axes[0].plot(d_list1)
    axes[0].plot(np.unwrap(d_list1))
    axes[0].set_ylabel('wrapped_phase (rad)')

    # print(d_list1)
    # print(np.unwrap(d_list1))

    axes[1].plot(d_list2, label='unwrapped_phase')
    axes[1].set_ylabel('unwrapped_phase (rad)')

    axes[2].plot(d_list3, label='distance from camera')
    axes[2].plot(phase2dist, label='distance calculated by phase')
    axes[2].set_ylabel('distance (m)')
    
    axes[3].plot(dist2phase, label='phase calculated by distance')
    axes[3].plot(modified_unwrapped_phase, label='modified unwrapped_phase')
    axes[3].set_ylabel('modified phase (rad)') # make positive/negative correlated
    axes[3].set_xlabel('samples')
    plt.legend()
    plt.show()
    # plt.savefig('1_0.23_wrapped.png')







if __name__ == '__main__':
    
    try:
        rospy.init_node('tag_positioning', anonymous = True)
        rate = rospy.Rate(100)

        # option 1
        while not rospy.is_shutdown():
            # rospy.Subscriber("/rfid_message", rfid_msg, lambda msg: rfid_callback(msg, phasor_unwrapped))

            rfid_subscriber = RFID_Subscriber()
            rate.sleep() # what usage?
            rospy.spin() # what usage?

        # option 2
        # rfid_subscriber = RFID_Subscriber()
        # rospy.spin()

        # file.close()

    except rospy.ROSInterruptException:
        pass
    
    


    ########### only for testing ##########
    # only_test()
    # read_csv()
    # plot_bi_compare()
    # plot_multi()



# extra work: putting the rosrun into roslaunch, and embedded the IP address
# coding on real-time variables illustration for convenient debug