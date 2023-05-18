import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
from rfid.msg import rfid_msg

# using message filter to subscribe 3 topics at the same time in 1 callback func

def callback(color, depth, rfid):
    bridge = CvBridge()
    
    # checking three outputs
    color_image = bridge.imgmsg_to_cv2(color, 'bgr8')
    depth_image = bridge.imgmsg_to_cv2(depth, '16UC1')
    
    cv2.imshow('color_image', color_image)
    cv2.imshow('depth_image', depth_image)
    print(rfid.epc)

    cv2.waitKey(1)


if __name__ == '__main__':
    # global fx, fy, ppx, ppy
    # fx = 609.134765 
    # fy = 608.647949
    # ppx = 312.763214
    # ppy = 240.882049
 
    rospy.init_node('sub_synchro', anonymous=True)
 
    img_color_info = message_filters.Subscriber("/camera/color/image_raw", Image)
    img_depth_info = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
    rfid_info = message_filters.Subscriber("/rfid_message", rfid_msg)

    # color_depth = message_filters.ApproximateTimeSynchronizer([color, depth], 10, 1, allow_headerless=True)  # approximate synchro
    combined_info = message_filters.TimeSynchronizer([img_color_info, img_depth_info, rfid_info], 1)  # absolute synchro, how is the last param?
    combined_info.registerCallback(callback)
    
    
    rospy.spin()

    

    # Problems pending solving:
    # rfid_msg has no attribute "header", including seq, stamp, frame_id
    # message_filter probably aligns the message according to header/stamp
    # add a header in rfid_msg

    # can it be three topics in message filter at the same time?