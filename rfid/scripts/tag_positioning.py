import rospy
import numpy as np
from cmath import exp
from rfid.msg import rfid_msg

import csv
import matplotlib.pyplot as plt


################### modified by Rong Zhiyi ####################

################## embedded in a config file ##################
TAG_ID = 'E280-1160-6000-0209-F811-48C3'
PHASE_LOAD_NUM = 4
###############################################################
# file = open("dist.txt", "a+")


###############################################################
# considering the single antenna first
# phase_loader consists of 4 consecutive phase (can be interrupted)

class RFID_Subscriber:
    def __init__(self):
        self.phasor_unwrapped = []
        self.phase_old = 0
        self.phase_current = 0
        self.subscriber = rospy.Subscriber('/rfid_message', rfid_msg, self.callback)
    
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
                    self.phasor_unwrapped.append(self.phasor_unwrapped[PHASE_LOAD_NUM-1] + self.phase_current - self.phase_old - 2 * np.pi)
                    # file.write(str(self.phasor_unwrapped[PHASE_LOAD_NUM-1] + self.phase_current - self.phase_old - 2 * np.pi) + "\n") ###
                elif self.phase_old - self.phase_current > np.pi:
                    self.phasor_unwrapped.append(self.phasor_unwrapped[PHASE_LOAD_NUM-1] + self.phase_current - self.phase_old + 2 * np.pi)
                    # file.write(str(self.phasor_unwrapped[PHASE_LOAD_NUM-1] + self.phase_current - self.phase_old + 2 * np.pi) + "\n") ###
                else:
                    self.phasor_unwrapped.append(self.phasor_unwrapped[PHASE_LOAD_NUM-1] + self.phase_current - self.phase_old)
                    # file.write(str(self.phasor_unwrapped[PHASE_LOAD_NUM-1] + self.phase_current - self.phase_old) + "\n") ###
                if len(self.phasor_unwrapped) > PHASE_LOAD_NUM:
                    del self.phasor_unwrapped[0] # phasor_unwrapped.pop(0)
                self.phase_old = self.phase_current

            # print('-------------------------------')
            # print(self.phasor_unwrapped)
            # print(self.phasor_normalized())


    def phasor_normalized(self):
        phasor_temp = []
        for i in range(PHASE_LOAD_NUM):
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
    file = open("dist.txt", "r")
    traj = file.readlines() # class str
    d_list = []

    for line in traj:
        # print(line)
        # str = line.split() # split by space, index 3 for x and index 5 for y
        d_list.append(float(line))


    # fig, axes = plt.subplots()

    plt.plot(d_list)
    # plt.xlabel('X of tf_translation (m)')
    plt.ylabel('Unwrapped Phase (rad)')
    # plt.title('data5')
    # plt.axis('equal')
    # plt.show()
    plt.savefig('unwrapped_phase.png')

















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
    # plot_d()



# extra work: putting the rosrun into roslaunch, and embedded the IP address