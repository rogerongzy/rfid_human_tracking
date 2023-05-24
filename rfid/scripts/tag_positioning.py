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
F_X, F_Y = 649.376, 649.376 # focal_length
C_X, C_Y = 648.137, 353.517 # principal_point
###############################################################
file1 = open("phase_wrap.txt", "a+")
file2 = open("phase_unwrap.txt", "a+")
file3 = open("dist_camera.txt", "a+")


###############################################################
# considering the single antenna first
# phase_loader consists of 4 consecutive phase (can be interrupted)

class RFID_Subscriber:
    def __init__(self):
        self.camera_init()

        self.phasor_unwrapped = []
        self.phase_old = 0 # none
        self.phase_current = 0 # none
        self.subscriber = rospy.Subscriber('/rfid_message', rfid_msg, self.callback)


    def camera_init(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) # depth
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) # color
        self.pipeline.start(self.config)
        self.tag_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))


    def tag_detect(self):
        frames = self.pipeline.wait_for_frames()
        # depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # depth_image = np.asanyarray(depth_frame.get_data()) # numpy.ndarray
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        # show images
        cv2.imshow('RealSense', color_image)
        key = cv2.waitKey(1)

        tags_list = self.tag_detector.detect(gray)
        for tags in tags_list:
            # add selectioin on tags
            if tags.tag_id == 0:
                pose, _, _ = self.tag_detector.detection_pose(tags, camera_params=(F_X, F_Y, C_X, C_Y), tag_size=TAG_SIZE)
                self.tag_position = pose[:3, 3]
                # rotation = pose[:3, :3]


    def camera_shutdown(self):
        self.pipeline.stop()


    
    def callback(self, msg):
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
            # if len(self.phasor_unwrapped) < PHASE_LOAD_NUM:
            #     self.phasor_unwrapped.append(msg.phase / 180 * np.pi)
            #     self.phase_old = msg.phase / 180 * np.pi
            # else:
                
                
            # update the phasor
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
                file2.write(str(self.phasor_unwrapped[-1]) + "\n")
                file1.write(str(self.phase_current) + "\n")
                
            print('-------------------------------')
            print(self.phasor_unwrapped)
            # print(self.phasor_normalized())

            self.tag_detect()
            file3.write(str(self.tag_position[2]) + "\n")
            print(self.tag_position[2])


    def phasor_normalized(self):
        phasor_temp = []
        for i in range(len(self.phasor_unwrapped)):
            phasor_temp.append(exp(-1j*(self.phasor_unwrapped[i] - self.phasor_unwrapped[0])))
        phasor_normalized = np.matrix(phasor_temp)
        # phasor_normalized = np.matrix([1, exp(-1j*(self.phasor_unwrapped[1] - self.phasor_unwrapped[0])), exp(-1j*(self.phasor_unwrapped[2] - self.phasor_unwrapped[0])), exp(-1j*(self.phasor_unwrapped[3] - self.phasor_unwrapped[0]))])
        return phasor_normalized












# def process():
#     # normal
#     # phasor_normalized = [1, exp(-1j*(phasor[1] - phasor[0])), exp(-1j*(phasor[2] - phasor[0])), exp(-1j*(phasor[3] - phasor[0]))]
#     # print(phasor_normalized) # can not show the complex using print(float)

#     # numpy.ndarray, shape(4,)
#     # phasor_normalized = np.array([1, exp(-1j*(phasor[1] - phasor[0])), exp(-1j*(phasor[2] - phasor[0])), exp(-1j*(phasor[3] - phasor[0]))])
#     # print(phasor_normalized)
#     # print(phasor_normalized.getH())

#     # numpy.matrix, shape(1,4)
#     phasor_normalized = np.matrix([1, exp(-1j*(phasor_unwrapped[1] - phasor_unwrapped[0])), exp(-1j*(phasor_unwrapped[2] - phasor_unwrapped[0])), exp(-1j*(phasor_unwrapped[3] - phasor_unwrapped[0]))])
#     # print(phasor_normalized)
#     # print(phasor_normalized.getH())

#     # complex matrix multiplication
#     # print(np.dot(phasor_normalized, phasor_normalized.getH()))

#     # 2-norm









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



def read_csv():
    phase1 = []
    count = 0
    # Open the CSV file
    with open('data.csv', 'r') as file:
        # Create a CSV reader object
        csv_reader = csv.reader(file)

        # Read the header (optional)
        # header = next(csv_reader)

        # Process each row in the CSV file
        for row in csv_reader:
            if count < 300:
                phase1.append(float(row[0]))
                count = count + 1

    fig, axes = plt.subplots(nrows=4, ncols=1)
    axes[0].plot(phase1)
    axes[1].plot(data_unwrap(phase1, 1.5))
    axes[2].plot(data_unwrap(phase1, 1)) # best
    axes[3].plot(data_unwrap(phase1, 0.25))
    plt.show()
    # plt.savefig('data_phase1.png')

    # plt.xlabel('X of tf_translation (m)')
    # plt.title('data5')
    # plt.axis('equal')



# offline unwrap method
def data_unwrap(array, scale): # scale=1 is chosen as the best constant
    current_layer = 0
    unwrap_array = []
    for i in range(len(array)-1):
        unwrap_array.append(array[i] + current_layer*2*np.pi)
        if array[i] - array[i+1] > scale*np.pi:
            current_layer = current_layer + 1
        elif array[i+1] - array[i] > scale*np.pi:
            current_layer = current_layer - 1
    return unwrap_array



def plot_d():
    file = open("wrap.txt", "r")
    dist = file.readlines() # class str
    d_list = []

    for line in dist:
        # print(line)
        # str = line.split() # split by space, index 3 for x and index 5 for y
        d_list.append(float(line))


    # fig, axes = plt.subplots()

    plt.plot(d_list)
    # plt.xlabel('X of tf_translation (m)')
    plt.ylabel('Wrapped Phase (rad)')
    # plt.title('data5')
    # plt.axis('equal')
    # plt.show()
    plt.savefig('wrapped_phase.png')



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
    d_list4 = []

    lmd = 0.162924707 * 2
    # lmd = 0.122924707 * 2

    for line1 in data1:
        d_list1.append(float(line1))
    for line2 in data2:
        d_list4.append(-float(line2) * lmd / (4 * np.pi) + float(data3[0]) + float(data2[0]) * lmd / (4 * np.pi)) # regularize with same initial
        d_list2.append(float(line2))
    for line3 in data3:
        d_list3.append(float(line3))

    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(20, 8))
    axes[0].plot(d_list1)
    axes[0].set_ylabel('wrapped_phase (rad)')
    axes[1].plot(d_list2)
    axes[1].set_ylabel('unwrapped_phase (rad)')
    axes[2].plot(d_list3, label='distance from camera')
    axes[2].plot(d_list4, label='distance calculated by phase')
    # axes[2].set_legend('camera_distance (m)')
    axes[2].set_ylabel('distance (m)')
    axes[2].set_xlabel('samples')
    # axes[3].plot(d_list4)
    # axes[3].set_ylabel('calculated_distance (m)')
    plt.legend()
    # plt.show()
    plt.savefig('pic_3in1.png')







if __name__ == '__main__':
    
    # try:
    #     rospy.init_node('tag_positioning', anonymous = True)
    #     rate = rospy.Rate(100)

    #     # option 1
    #     while not rospy.is_shutdown():
    #         # rospy.Subscriber("/rfid_message", rfid_msg, lambda msg: rfid_callback(msg, phasor_unwrapped))

    #         rfid_subscriber = RFID_Subscriber()
    #         rate.sleep() # what usage?
    #         rospy.spin() # what usage?

    #     # option 2
    #     # rfid_subscriber = RFID_Subscriber()
    #     # rospy.spin()

    #     # file.close()

    # except rospy.ROSInterruptException:
    #     pass
    
    


    ########### only for testing ##########
    # only_test()
    # read_csv()
    # plot_d()
    plot_multi()



# extra work: putting the rosrun into roslaunch, and embedded the IP address