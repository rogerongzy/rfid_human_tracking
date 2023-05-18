import statistics
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
# matplotlib.use('TkAgg')
from rfid.rfid_utils.basic import *
from scipy.signal import argrelmax
from scipy.stats import multivariate_normal

def rad2deg(rad):
    return rad*180/np.pi

def phase_wrap(phase):
    phase -= np.pi
    while abs(phase)>np.pi:
        if phase >= 0:
            phase -=  2 * np.pi
        else:
            phase +=  2 * np.pi
    return phase + np.pi

def fusion_SAR_simplify(phase_received, offset = [0, np.pi +0.5, np.pi+1, -0.5 ], antenna_enable = [1,1,1,1],num_of_search = 200):
    antenna_list_candidate = ['1', '2', '3', '4']
    antenna_list = []
    for idx, antenna in  enumerate(antenna_list_candidate):
        if antenna_enable[idx] == 1:
            antenna_list.append(antenna)
                
    phase_received_new = np.zeros(4)
    for idx,phase in enumerate(phase_received):
        phase_received_new[idx] = phase_wrap(phase - offset[idx])

    wave_len = 0.1629247078010329 * 2
    half_wave_len = 0.1629247078010329
    D_list = {'1': 1.5, '2': 0.5, '3':-0.5, '4': -1.5} 
    aoa_range = np.linspace(0,np.pi,num_of_search)
    SAR_graph_2 = np.zeros(len(aoa_range))
    for i, aoa in enumerate(aoa_range):
        phase_list = []
        for k, antenna in enumerate(antenna_list):
            antenna_idx = int(antenna)-1
            phase_received_temp = phase_received_new[antenna_idx]
            aoa_estimated_temp = np.cos(aoa) * 0.25 * 1 * D_list[antenna] * 2 * np.pi / half_wave_len
            phase_list.append(phase_received_temp - aoa_estimated_temp)
        _,SAR_graph_2[i] = holograph_addon(phase_list)


    return SAR_graph_2

