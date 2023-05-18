# !/usr/bin/env python
# -*- coding: utf-8 -*-
import struct
import math
import numpy as np
import time
import re
import rospy
from rfid.msg import rfid_msg
from dataloader_robot import *
from std_msgs.msg     import Float64
import actionlib
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import apriltag
global goal
import pyrealsense2 as rs

f = open('/home/linzp/Desktop/trajectory.txt','w')



class Rfid:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.color_frame = None
        self.gray = None
        self.get_image()
        self.rfid_position()

    def get_image(self):

        config = rs.config()
        pipe = rs.pipeline()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        pipe.start(config)
        frameset = pipe.wait_for_frames()
        align = rs.align(rs.stream.color)
        frameset = align.process(frameset)

        self.color_frame = frameset.get_color_frame()
        color_image = np.asanyarray(self.color_frame.get_data())
        self.gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('frame.jpg', color_image)
        #cv2.waitKey(0)
        # print(color_image)
        
        return self.gray

    
    def rfid_position(self):

        tag_size = 0.16
        fx, fy = 649.376, 649.376
        cx, cy = 648.137, 353.517
        cam_params = (fx, fy, cx, cy)
        K = [[fx, 0, cx],
             [0, fy, cy],
             [0, 0, 1]]

        detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9'))
        
        results = detector.detect(self.gray)
        print("[INFO] {} total AprilTags detected".format(len(results)))

        for tag in results:
            '''
            cv2.circle(self.gray, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
            cv2.circle(self.gray, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
            cv2.circle(self.gray, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
            cv2.circle(self.gray, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
            '''
        #cv2.imshow('capture', self.gray)
        #cv2.waitKey(0)
            pose, _, _ = detector.detection_pose(tag, cam_params, tag_size)
            tvec = pose[0:3, 3]
            print(f"World Coordinates: {tvec}")
        # return tvec
    
class Phase:
    def __init__(self):
        self.previous_raw_phase = None
        self.initial_phase = None
        self.raw_phase = 0
        self.phase_unwrap_previous = 0
        rospy.Subscriber("/rfid_message", rfid_msg, self.callback)

    def unwrap(self):

        if self.raw_phase - self.previous_raw_phase >= np.pi:
            phase_unwrap = self.phase_unwrap_previous + (self.raw_phase - np.pi * 2 - self.previous_raw_phase)
        elif self.raw_phase - self.previous_raw_phase <= -np.pi:
            phase_unwrap = self.phase_unwrap_previous + (self.raw_phase + np.pi * 2 - self.previous_raw_phase)
        else:
            phase_unwrap = self.phase_unwrap_previous + (self.raw_phase - self.previous_raw_phase)
        self.phase_unwrap_previous = phase_unwrap
        return phase_unwrap

    def callback(self, data):
        # [Timestamp    Antenna     RFID     Freq    RSSI    Phase] #
        # newline = [data.time / 1000000, data.ant, data.epc, 0, data.rssi, data.phase / 180 * np.pi]rosrun rfid rfid 169.254.1.1
        if data.ant == 1 and data.epc =='E280-1160-6000-0209-F811-48C3':
            #print(data.ant)
            phase = data.phase/ 180 * np.pi
            if self.initial_phase is None:
                self.initial_phase = phase
                self.previous_raw_phase = phase
                self.phase_unwrap = phase
                phase_unwrap = phase
                self.phase_unwrap_previous = phase_unwrap
                print("phase_unwrap",phase_unwrap)

            else:
                self.raw_phase = phase
                phase_unwrap = self.unwrap()
                self.phase_unwrap_previous = phase_unwrap
                self.previous_raw_phase = phase
                #print("phase_unwrap###",phase_unwrap)
            #f.write(str(phase_unwrap)+'\n')
            return phase_unwrap
        

def match(a,b):
    conjugate_array = np.conjugate(a)
    trans_array = conjugate_array.transpose()
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)
    C = abs(np.dot(trans_array,b))/(a_norm*b_norm)
    return Cpipe.start(config)

def complex_sequence(vec,Nr):
    e = []
    for i in range(Nr):
        if i == 0:
            e.append(0)
        else:
            e.append(np.exp(-1j*(vec[i]-vec[i-1])))
    return e

if __name__ == '__main__':
    try:
        wave_len = 0.1629247078010329 * 2
        Nr = 8
        Phase_vec = []
        Phase_save = []
        rospy.init_node("rfid_localization")
        rospy.loginfo("rfid_localization")
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            # rospy.Subscriber("/rfid_message", rfid_msg, rfid_callback)
            rfid_pos = Rfid()
            Phase_value = Phase()
            
            if len(Phase_vec)<Nr:
                Phase_vec.append(Phase_value)
            else:
                del(Phase_vec[0])
                Phase_vec.append(Phase_value)
            
            rate.sleep()
            rospy.spin()

    except rospy.ROSInterruptException:
        pass

