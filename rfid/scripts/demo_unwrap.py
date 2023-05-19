# import rospy
# from std_msgs.msg import String
# from rfid.msg import rfid_msg
# import numpy as np
import apriltag
import pyrealsense2 as rs
import numpy as np
import cv2


 
if __name__ == "__main__":
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) # depth
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) # color
    # Start streaming
    pipeline.start(config)



    # img = cv2.imread('frame.jpg')
    # print(type(img))
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    tag_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9'))
    

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            # depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            # if not depth_frame or not color_frame:
            if not color_frame:
                continue
            # Convert images to numpy arrays
 
            # depth_image = np.asanyarray(depth_frame.get_data())
 
            color_image = np.asanyarray(color_frame.get_data()) # numpy.ndarray
            # print(type(color_image))
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # Stack both images horizontally
            # images = np.hstack((color_image, depth_colormap))

            # Show images
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            key = cv2.waitKey(1)
            tags_pos_list = tag_detector.detect(gray)
            print('--------------------------------')
            print(tags_pos_list)


            # Press esc or 'q' to close the image window
            # if key & 0xFF == ord('q'):
            if False:
                cv2.destroyAllWindows()
                break

    finally:
        pipeline.stop()



    # frames = pipeline.wait_for_frames()
    # color_frame = frames.get_color_frame()
    # color_image = np.asanyarray(color_frame.get_data())
    # cv2.imshow('RealSense', color_image)
    # cv2.waitKey(0)
        
    # # Stop streaming
    # pipeline.stop()