# import rospy
# from std_msgs.msg import String
# from rfid.msg import rfid_msg
# import numpy as np
import apriltag

# class MySubscriber:
#     def __init__(self, param1, param2):
#         self.param1 = param1
#         self.param2 = param2
#         self.subscriber = rospy.Subscriber('/rfid_message', rfid_msg, self.callback)
    
#     def callback(self, msg):
#         # Access the additional parameters
#         self.param1 = self.param1 + 1
#         print(self.param1, self.param2)
#         # Process the message
#         print(msg)

# def main():
#     rospy.init_node('my_node')
#     param1 = 123
#     param2 = 'hello'
#     my_subscriber = MySubscriber(param1, param2)
#     rospy.spin()

# if __name__ == '__main__':
#     main()


at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9'))


# for i in range(dataset_music.shape[0]):
#     if i == 0:
#         phase_online[0,0]=dataset_music[0,0]

#     else:
#         if dataset_music[i,0]-dataset_music[i-1,0] >= np.pi:
#             phase_online[i, 0] = phase_online[i-1, 0] + (dataset_music[i,0]-np.pi*2 - dataset_music[i-1,0])

#         elif dataset_music[i,0]-dataset_music[i-1,0] <=-np.pi:

#             phase_online[i, 0] = phase_online[i-1, 0] + (dataset_music[i,0]+np.pi*2 - dataset_music[i-1,0])

#         else:
#             phase_online[i, 0] = phase_online[i-1, 0] + (dataset_music[i, 0]-dataset_music[i-1, 0])



# pyplot.plot(np.array(phase[1:300, 0]),color='red', label='b')
# pyplot.plot(np.array(phase_online[1:300, 0]),color='blue', label='b')
# pyplot.show()


