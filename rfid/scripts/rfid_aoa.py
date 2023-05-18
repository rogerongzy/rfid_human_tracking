import struct
import math
import numpy as np
import time
import re
import rospy
import cv2
from rfid.msg import rfid_msg
from sensor_msgs.msg import Image
from dataloader_robot import *
from std_msgs.msg     import Float64
from cv_bridge import CvBridge, CvBridgeError


phase_calib = np.array([0.57, -1.20, -0.97, 0.31, -0.17, 0.41])   
#phase_calib = np.array([0, -1.40])
NUM_OF_ANT = 6
INTERVAL = 0.1 # 0.25

def ULA_aoa_spectrum(phase, interval = INTERVAL, Num_ant = NUM_OF_ANT, res = 200):
    '''
        input args:
            phase:     batch_size, Number of antennas
            cv_aoa:    batch_size,
        output args:
            spectrum:  batch_size, resolution (200)
    '''
    wave_len = 0.1629247078010329 * 2
    half_wave_len = 0.1629247078010329
    aoas = np.linspace(0, np.pi, res)
    steering_vector = np.zeros((res, Num_ant),dtype=np.complex_)
    phi = 2 * np.pi / half_wave_len * interval * np.cos(aoas) * -1  # should be same to ULA_aoa_calibration

    for i in range(Num_ant):
        steering_vector[:,i] = np.cos(phi * i) + np.sin(phi * i) * 1j

    phase_complex = np.zeros((np.shape(phase)[0], Num_ant),dtype=np.complex_)
    for i in range(Num_ant):
        phase_complex[:,i] = np.cos(phase[:,i]) + np.sin(phase[:,i]) * 1j

    spectrum = np.zeros((np.shape(phase)[0], res))
    for i in range(np.shape(phase)[0]):
        spectrum_new = np.matmul((phase_complex[i,:]), steering_vector.T)
        spectrum[i,:] = abs(spectrum_new)
    spectrum /= Num_ant
    return spectrum

def rfidcallback(data):
	# [Timestamp    Antenna     RFID     Freq    RSSI    Phase] #
	newline = [data.time/1000000, data.ant, data.epc, 0, data.rssi, data.phase / 180 * np.pi ]
	#print(newline)
	start_time, stop_time, data_valid = RFID_data.update(newline)
	current_time = time.time()
	delay = data.time/1000000-current_time
	print('Delay',delay)
	print('-----------datavalid', data_valid)
	#print(start_time, stop_time, valid)
	runtime_flag = delay >= 80 # initail value should be around 81 seconds
	if data_valid and runtime_flag:
		phase_sampled,rssi_sampled = RFID_data.get_data_6A((stop_time + start_time ) /2)
		spectrum = ULA_aoa_spectrum(np.array([phase_sampled]) + np.expand_dims(phase_calib, axis = 0) , interval = INTERVAL, Num_ant = NUM_OF_ANT, res = 200)[0,:]
 		#print(phase_sampled[0]-phase_sampled[1])
		#aoa_pred = (np.argmax(spectrum[40:160]) + 40) / 200
		aoa_pred = (np.argmax(spectrum)) / 200
		if np.max(spectrum) > 0:## 1 threshold
			aoa_pred_transformed = (1 - aoa_pred) * 180 
		else:
			aoa_pred_transformed = None
	else:
		phase_sampled,rssi_sampled = None, None
		aoa_pred_transformed = None
		
	#print(phase_sampled,rssi_sampled)
	#print(spectrum)
	#print(start_time, stop_time, valid)
	#print('1', data.time)
	#print('2', time.time())
	print('AOA_pred',aoa_pred_transformed)
	pub.publish(aoa_pred_transformed)


def rad2deg(rad):
	return rad / (2 * np.pi) * 180
	
def deg2rad(deg):
	return deg / 180 * 2 * np.pi

	
def imagecallback(image_data):
	try:
		cv_image = bridge.imgmsg_to_cv2(image_data, 'bgr8')
		# print(np.shape(cv_image))
	
		# cv2.destroyAllWindows()
		# print('size', np.shape(cv_image))
		print('glob var', aoa_pred_transformed)
		u_pix = AoA_to_angle_without_distortion_correct(np.tan(deg2rad(aoa_pred_transformed)))
		cv2.line(cv_image, (int(u_pix), 0), (int(u_pix), 480), (255,0,0), 2)
		
		cv2.imshow('view', cv_image)
		cv2.waitKey(1)
		
	except CvBridgeError as e:
		print 
		e


def AoA_to_angle_without_distortion_correct(tan_theta):
	fx = 386.4058837890625 # realsense_len
	u_0 = 320.6568298339844 # central pixel
	u = fx * tan_theta + u_0
	return u


if __name__ == '__main__':
   
	Tag_ID = 'E280-1160-6000-0209-F811-72F3' # NO 5
	RFID_data = RFID_dataloader_robot(Tag_ID, num_of_antenna = NUM_OF_ANT) # ant_num

	try:
		rospy.init_node('rfid_aoa', anonymous = True)
		
		rate = rospy.Rate(100)
		global aoa_pred_transformed
		aoa_pred_transformed = 0
		pub = rospy.Publisher('/AoA', Float64, queue_size=10)
		global bridge
		bridge = CvBridge()
		
		past_time = time.time()
		while not rospy.is_shutdown():
			
			current_time = time.time()
			#print('loop_interval:',current_time-past_time)
			past_time = current_time
			
			
			rospy.Subscriber("/rfid_message", rfid_msg, rfidcallback)
			rospy.Subscriber('/camera/color/image_raw', Image, imagecallback) # 

			rate.sleep() #
			rospy.spin()
			
		
	except rospy.ROSInterruptException:
		pass
