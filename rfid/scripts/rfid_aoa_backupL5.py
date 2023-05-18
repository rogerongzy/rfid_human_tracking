import struct
import math
import numpy as np
import time
import re
import rospy
from rfid.msg import rfid_msg
from dataloader_robot import *
from std_msgs.msg     import Float64

phase_calib = np.array([0.57, -1.20, -0.97, 0.31, -0.17, 0.41])
def ULA_aoa_spectrum(phase, interval = 0.1, Num_ant = 6, res = 200):
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
	start_time, stop_time, valid = RFID_data.update(newline)
	if valid:
		phase_sampled,rssi_sampled = RFID_data.get_data_6A((stop_time + start_time ) /2)
		spectrum = ULA_aoa_spectrum(np.array([phase_sampled]) + np.expand_dims(phase_calib, axis = 0) , interval = 0.1, Num_ant = 6, res = 200)[0,:]
		aoa_pred = (np.argmax(spectrum[40:160]) + 40) / 200
		if np.max(spectrum) > 0.7:
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
	
if __name__ == '__main__':
	
	
	RFID_data = RFID_dataloader_robot('E280-1160-6000-0209-F811-5CD4', num_of_antenna = 6)
	
	try:
		
		rospy.init_node('rfid_aoa', anonymous = True)
		rate = rospy.Rate(10)
		aoa_pred_transformed = 0
		pub = rospy.Publisher('/AoA', Float64, queue_size=10)
		while not rospy.is_shutdown():
			
			
			rospy.Subscriber("/rfid_message", rfid_msg, rfidcallback)
			rate.sleep()
			#rospy.spin()
			
			
			
			
		
	except rospy.ROSInterruptException:
		pass
