import numpy as np

import rospy
from cmath import exp
from rfid.msg import rfid_msg

import csv
import matplotlib.pyplot as plt


############# embedded in a config file #############
TAG_ID = 'E280-1160-6000-0209-F811-48C3'
PHASE_LOAD_NUM = 4
#####################################################
phase_vector = []
layer = []
phase_vector_unwrapped = []






################################################################
# considering the single antenna first
# phase_loader consists of 4 consecutive phase (can be interrupted)


def rfid_callback(data):
    ### [Timestamp    Antenna     RFID     Freq    RSSI    Phase] #
    ### newline = [data.time/1000000, data.ant, data.epc, 0, data.rssi, data.phase / 180 * np.pi ]
    
    # assumed the signals are good enough, directly put into phase_vector
    # need unwrap before put into phase_loader
    if data.epc == TAG_ID and data.ant == 1:

        if len(phase_vector) < PHASE_LOAD_NUM:
            phase_vector.append(data.phase / 180 * np.pi)
        elif len(phase_vector) == PHASE_LOAD_NUM:
            # update the value of phase_vector
            # phase_vector[0] = phase_vector[1]
            # phase_vector[1] = phase_vector[2]
            # phase_vector[2] = phase_vector[3]
            # phase_vector[3] = data.phase / 180 * np.pi
            phase_vector = phase_vector[1:len(phase_vector)]
            phase_vector.append(data.phase / 180 * np.pi)

            # unwrap(real-time version)
            



            process()


    
    # timely print, see what will show in 1 timestamp
    # time.sleep(1)
    # print(data)



def process():
    # normal
    # phase_vector_normalized = [1, exp(-1j*(phase_vector[1] - phase_vector[0])), exp(-1j*(phase_vector[2] - phase_vector[0])), exp(-1j*(phase_vector[3] - phase_vector[0]))]
    # print(phase_vector_normalized) # can not show the complex using print(float)
    
    # numpy.ndarray, shape(4,)
    # phase_vector_normalized = np.array([1, exp(-1j*(phase_vector[1] - phase_vector[0])), exp(-1j*(phase_vector[2] - phase_vector[0])), exp(-1j*(phase_vector[3] - phase_vector[0]))])
    # print(phase_vector_normalized)
    # print(phase_vector_normalized.getH())

    # numpy.matrix, shape(1,4)
    phase_vector_normalized = np.matrix([1, exp(-1j*(phase_vector[1] - phase_vector[0])), exp(-1j*(phase_vector[2] - phase_vector[0])), exp(-1j*(phase_vector[3] - phase_vector[0]))])
    # print(phase_vector_normalized)
    # print(phase_vector_normalized.getH())

    # complex matrix multiplication
    print(np.dot(phase_vector_normalized, phase_vector_normalized.getH()))
    
    # take the 2-norm









########################### testing ##################################
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
        

    

















if __name__ == '__main__':
    
    # try:
    #     rospy.init_node('rfid_localization', anonymous = True)
    #     rate = rospy.Rate(100)
        
    #     while not rospy.is_shutdown():
    #         rospy.Subscriber("/rfid_message", rfid_msg, rfid_callback)
            
    #         rate.sleep()
    #         rospy.spin()
        
    # except rospy.ROSInterruptException:
    #     pass
    



    ########### only for testing ##########
    # only_test()
    read_csv()



# extra work: putting the rosrun into roslaunch, and embedded the IP address