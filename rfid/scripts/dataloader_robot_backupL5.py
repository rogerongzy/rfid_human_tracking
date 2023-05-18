## for 6 Antenna

## load data from ros topic
## input: sequential data
## return: resampled phase and RSSI data 
## currently only surport single RFID tag
## online processing

import time
import datetime
import math
import csv
import numpy as np
from rfid_utils.basic import *
# from rfid.rfid_utils.fusion_SAR import *
def phase_wrap(phi):
    phi -= np.pi
    while abs(phi) > np.pi:
        if phi > 0:
            phi -= 2 * np.pi
        else:
            phi += 2 * np.pi
    return phi + np.pi
            

class RFID_dataloader_robot:
    def __init__(self, rfid_wanted, num_of_antenna):#rfid_list_wanted = ['E280 1160 6000 0209 F811 7224'] #  ['E280 1160 6000 0209 F811 7224']['E280 6894 0000 500E 73D4 548C']
        self.det_data = []
        self.antenna_list = []
        self.rfid_list = []
        self.data_pool = []
        self.num_of_antenna = num_of_antenna
        self.data_pool_max_len = 100 # length of the pool. if too much, then consume time
        self.rfid_wanted = rfid_wanted

    def update(self, newline):
    # check the newline is not belong to date pool
        if self.data_pool == []:
            self.data_pool.append(newline)
        else:
            if newline[2] == self.rfid_wanted:
                if newline != self.data_pool[-1]:
                    self.data_pool.append(newline)
                    
        while len(self.data_pool) > self.data_pool_max_len:
            self.data_pool.pop(0)
        self.det_data = []
        self.antenna_list = []
        self.rfid_list = []

        ###  RFID csv file   ###
        # [Timestamp    Antenna     RFID     Freq    RSSI    Phase] #

        for row in self.data_pool:
            if row[2] == self.rfid_wanted:
                if row[2] not in self.rfid_list and row[2] != '':
                    self.rfid_list.append(row[2])
                if row[1] != '':
                    if row[1] not in self.antenna_list:
                        self.antenna_list.append(row[1])
                        self.det_data.append([])
                    self.det_data[self.antenna_list.index(row[1])].append(row)

        ## basic operation ## 
        ####phase ==> [antenna idx][RFid]==>[phase0,phase1,phase2...]
        ####ptime ==> [antenna idx][RFid]==>[time0,time1,time2...]

        self.phase = [[[] for j in self.rfid_list] for i in self.antenna_list]
        self.ptime = [[[] for j in self.rfid_list] for i in self.antenna_list]
        self.rssi = [[[] for j in self.rfid_list] for i in self.antenna_list]
        for i, antenna in enumerate(self.antenna_list):
            for j, rfid in enumerate(self.rfid_list):
                self.phase[i][j] = np.array([k[5] for k in self.det_data[self.antenna_list.index(antenna)] if k[2] == rfid])
                self.ptime[i][j] = np.array([k[0] for k in self.det_data[self.antenna_list.index(antenna)] if k[2] == rfid])
                self.rssi[i][j] = np.array([k[4] for k in self.det_data[self.antenna_list.index(antenna)] if k[2] == rfid])

        ## data validity check
        valid = True
        for i in range(len(self.antenna_list)):
            if len(self.phase[i][0]) <= 1:
                valid = False
        if len(self.antenna_list) < self.num_of_antenna:
            valid = False
            
        if not valid:
            return None, None, valid

        return np.max([self.ptime[i][0][0] for i in range(len(self.antenna_list))]), np.min([self.ptime[i][0][-1] for i in range(len(self.antenna_list))]), valid

    def get_data_6A(self, time_temp): ## get ONE data 

        T_polyfit = 0
        T_valid = 0.2
        order = 2 # linear spline interpolation
        self.phase_fit = []
        self.rssi_fit = []

        self.a_1_idx = self.antenna_list.index(1)
        self.a_2_idx = self.antenna_list.index(2)
        self.a_3_idx = self.antenna_list.index(3)
        self.a_4_idx = self.antenna_list.index(4)
        self.a_5_idx = self.antenna_list.index(5)
        self.a_6_idx = self.antenna_list.index(6)

        # phase polynomial regression
        for i, antenna in enumerate(self.antenna_list):    
            idx_f_i, idx_l_i = BinarySearch(self.ptime[i][0], time_temp)
            if len(self.ptime[i][0][idx_f_i:idx_l_i]) == 0:
                self.phase_fit.append(0)
            else:
                phase_temp = phase_wrap(holograph_poly_fit(self.ptime[i][0][idx_f_i:idx_l_i], self.phase[i][0][idx_f_i:idx_l_i], time_temp, 0, order = order, new_x = True))[0]
                self.phase_fit.append(phase_wrap(phase_temp))
        # rssi linear fitting
            if len(self.rssi[i][0][idx_f_i:idx_l_i]) == 0:
                self.rssi_fit.append(-80)
            else:
                self.rssi_fit.append(Linear_fitting_2pnts(self.ptime[i][0][idx_f_i], self.ptime[i][0][idx_l_i], self.rssi[i][0][idx_f_i], self.rssi[i][0][idx_l_i], time_temp))
        
        self.phase_list = [self.phase_fit[self.a_1_idx], self.phase_fit[self.a_2_idx], self.phase_fit[self.a_3_idx], self.phase_fit[self.a_4_idx] , self.phase_fit[self.a_5_idx], self.phase_fit[self.a_6_idx]]
        self.rssi_list = [self.rssi_fit[self.a_1_idx], self.rssi_fit[self.a_2_idx], self.rssi_fit[self.a_3_idx], self.rssi_fit[self.a_4_idx] , self.rssi_fit[self.a_5_idx], self.rssi_fit[self.a_6_idx]]
        return self.phase_list, self.rssi_list

    def get_data_4A(self, time_temp): ## get ONE data 

        T_polyfit = 0
        T_valid = 0.2
        order = 2 # linear spline interpolation
        self.phase_fit = []
        self.rssi_fit = []

        self.a_1_idx = self.antenna_list.index(1)
        self.a_2_idx = self.antenna_list.index(2)
        self.a_3_idx = self.antenna_list.index(3)
        self.a_4_idx = self.antenna_list.index(4)

        # phase polynomial regression
        for i, antenna in enumerate(self.antenna_list):
            _, idx_f_i = BinarySearch(self.ptime[i][0], time_temp - T_polyfit / 2)
            idx_l_i, _ = BinarySearch(self.ptime[i][0], time_temp + T_polyfit / 2)
            if len(self.ptime[i][0][idx_f_i:idx_l_i]) == 0:
                self.phase_fit.append(0)
            else:
                phase_temp = phase_wrap(holograph_poly_fit(self.ptime[i][0][idx_f_i:idx_l_i], self.phase[i][0][idx_f_i:idx_l_i], time_temp, 0, order = order, new_x = True))[0]
                self.phase_fit.append(phase_wrap(phase_temp))
        # rssi linear fitting
        for i, antenna in enumerate(self.antenna_list):    
            idx_f_i, idx_l_i = BinarySearch(self.ptime[i][0], time_temp)
            if len(self.rssi[i][0][idx_f_i:idx_l_i]) == 0:
                self.rssi_fit.append(-80)
            else:
                self.rssi_fit.append(Linear_fitting_2pnts(self.ptime[i][0][idx_f_i], self.ptime[i][0][idx_l_i], self.rssi[i][0][idx_f_i], self.rssi[i][0][idx_l_i], time_temp))
        
        self.phase_list = [self.phase_fit[self.a_1_idx], self.phase_fit[self.a_2_idx], self.phase_fit[self.a_3_idx], self.phase_fit[self.a_4_idx]]
        self.rssi_list = [self.rssi_fit[self.a_1_idx], self.rssi_fit[self.a_2_idx], self.rssi_fit[self.a_3_idx], self.rssi_fit[self.a_4_idx]]
        return self.phase_list, self.rssi_list

if __name__ == '__main__':
    RFID_data = RFID_dataloader_robot('E280 1160 6000 0209 F811 72C4', num_of_antenna = 4)
    with open( '/home/linzp/Data/RFID_CV_data/9.10/rfid/test1.csv', 'r') as f:
            f_csv = csv.reader(f)
            headers = next(f_csv)
            for row in f_csv:
                time1 = time.time()
                start_time, stop_time, valid = RFID_data.update(row)
                if valid:
                    _,_ = RFID_data.get_data_4A(start_time * 0.5 + stop_time * 0.5)
                    print(stop_time - start_time)
                time2 = time.time()
                print(time2-time1, valid)
                #print(start_time, stop_time)
            f.close()

