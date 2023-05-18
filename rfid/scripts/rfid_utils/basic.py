
import os
import time
from pathlib import Path
import datetime
import math
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib


def CV_preprocess(data_seq,time_seq):
    # simple preprocess, needs improve
    window_size = 10 # pnts
    threshold = 1 # meter
    delete_idx = []
    for idx in range(len(data_seq)-window_size):
        if abs(data_seq[idx]-data_seq[idx + window_size]) > threshold:
            delete_idx.append(idx + window_size)
            #data_seq[idx + window_size] = data_seq[idx]
    length = len(data_seq) - window_size
    data_seq_out = [data_seq[i] for i in range(length) if i not in delete_idx]
    time_seq_out = [time_seq[i] for i in range(length) if i not in delete_idx]
    return data_seq_out * 1, time_seq_out * 1

def BinarySearch(array,t):
    low = 0
    height = len(array)-1
    if array[low] > t:
        return low,low
    if array[height]<t:
        return height, height
    while low < height - 1:
        mid = (low+height) // 2
        if array[mid] == t:
            return mid-1, mid
        elif array[mid] < t:
            low = mid 
        elif array[mid] > t:
            height = mid 
    return low, height

def find_segment(time_first, time_last, time_seq):
    idx_first = 0
    idx_last = len(time_seq) - 1
    GT_tf = False
    GT_tl = False
    m = len(time_seq) 
    for idx, t in enumerate(time_seq):
        if not GT_tf and t > time_first:
            idx_first = idx
        if idx == m - 1 and t < time_first:
            idx_first = idx
        if not GT_tl and t > time_last:
            idx_last = idx
            break
        GT_tf = t > time_first
        GT_tl = t > time_last
    error = idx_first == idx_last or idx_first == 0

    return np.maximum(idx_first - 1,0), np.maximum(idx_last - 1,0), error

def convert(time,phase, resetTime = 0.2,jumpWindow = 4.5):
    # resetTime = 0.2  # in second
    # jumpWindow = 4.5  # in second
    new_phase=[phase[0]]
    for i in range(1,len(time)):
        if time[i] - time[i-1] > resetTime:
            new_phase.append(0)
        else:
            difference = phase[i] - phase[i-1] if abs(
                phase[i] - phase[i-1]) < jumpWindow else math.copysign(
                2 * math.pi, -phase[i] + phase[i-1]) + phase[i] - phase[i-1]
            new_phase.append(new_phase[-1]+difference)
    new_phase_output = new_phase * 1
    return new_phase_output

def linear_itp(timex,timey_seq,datay_seq):
    idx = 10000
    data_invalid_timegap = 1.1 # sec
    status = 3 #[0,1,2] ==> [before, now, after] 
    if timey_seq[0] >= timex:
        idx = 0
        status = 0
    elif timey_seq[-1] <= timex:
        idx = len(timey_seq)-1
        status = 2
    else:
        for i, timey in enumerate(timey_seq):
            if timey <= timex and timey_seq[i+1] >= timex:
                idx = i
                status = 1
                break

    if idx == 10000 or status == 3:
        print('ERROR',timex)
        raise ValueError

    if status == 0:
        output_value = datay_seq[0]
        if timey_seq[0] - timex < data_invalid_timegap:
            data_valid = True
        else:
            data_valid = False
    elif status == 2:
        output_value = datay_seq[-1]
        if timex - timey_seq[-1] < data_invalid_timegap:
            data_valid = True
        else:
            data_valid = False
    else:
        timey1 = timey_seq[idx]
        datay1 = datay_seq[idx]
        timey2 = timey_seq[idx + 1]
        datay2 = datay_seq[idx + 1]
        alpha_itp = (timex-timey1)/(timey2-timey1)
        output_value = datay1 * alpha_itp + datay2 * (1 - alpha_itp)
        data_valid = True
    return idx,  output_value, data_valid

def MLE_sect(seqx,seqy):
    # can ues gradient decent
    if len(seqx) != len(seqy):
        print('ERROR in MLE')
    bias_0 = seqy[0] - seqx[0]
    SE_temp = 100000
    bias_output = 100000
    range_search = np.linspace(-3,3,300)
    for i in range(len(range_search)):
        bias = range_search[i]
        error_temp = (seqx + bias + bias_0 - seqy) 
        if SE_temp > np.sum(np.dot(error_temp, error_temp)):
            SE_temp = np.sum(np.dot(error_temp, error_temp))
            bias_output = bias + bias_0
    MSE_output = SE_temp / len(seqx)

    return bias_output, MSE_output

def gradient_decent(seqx,seqy):
    if len(seqx) != len(seqy):
        print('ERROR in gd')
    bias_0 = seqy[0] - seqx[0]
    bias = 0
    delta_bias = 0.005
    epsilon = 0.001
    alpha = 0.4
    max_itor = 100
    cnt = 0
    m = len(seqx)
    
    while 1:
        cnt = cnt + 1
        error_temp_0 = np.array([seqx[i] - seqy[i] + bias + bias_0 + delta_bias for i in range(m)])
        SE_temp_0 = np.sum(np.dot(error_temp_0, error_temp_0))
        MSE_temp_0 = SE_temp_0 / m
        error_temp_1 = np.array([seqx[i] - seqy[i] + bias + bias_0 - delta_bias for i in range(m)])
        SE_temp_1 = np.sum(np.dot(error_temp_1, error_temp_1))
        MSE_temp_1 = SE_temp_1 / m
        grad = (MSE_temp_1 - MSE_temp_0) / (delta_bias * 2)
        #print('bias :%f,MSE :%f,grad :%f'%(bias,MSE_temp_0,grad))
        if abs(grad) < epsilon:
            break
        if cnt >= max_itor:
            print('cnt>max_itor')
            break
        bias += alpha * grad
    #print('bias :%f,MSE :%f,grad :%f'%(bias,MSE_temp_0,grad))
    return  (bias + bias_0), (MSE_temp_0 + MSE_temp_1) / 2

def GLRT(ptime, pdis, ctime, cdis, brkpnts):
    MSE_sect = []
    time_sect = []
    grouppnt_sect = []
    bias_sect = []
    pdis_glrt = pdis * 1
    error_sect = []
    idx_sect = []
    min_point = 4
    #brkpnts[0] = 1
    #brkpnts[-1] = 1
    for i,brkpnt in enumerate(brkpnts):
        if brkpnt == 1:
            try:
                next_i = brkpnts.index(1,i+1)
            except ValueError:
                next_i = len(brkpnts) - 1

            if next_i-i >= min_point:
                time_sect.append((ptime[i]))#(ptime[next_i] + ptime[i] )/2
                grouppnt_sect.append(next_i - i)
                #MLE
                cdis_itp_sect = []
                data_valid = True 
                for j in range(i,next_i):
                    #print(j,linear_itp(ptime[j],ctime,cdis))
                    a,b,flag = linear_itp(ptime[j],ctime,cdis)
                    cdis_itp_sect.append(b)
                    if not flag:
                        data_valid = False
                  
                if data_valid:
                    bias_hat, MSE_temp = gradient_decent(pdis[i:next_i],cdis_itp_sect)
                else:
                    bias_hat = 0
                    MSE_temp = 1
                error_sect.append(data_valid)
                MSE_sect.append(MSE_temp)
                bias_sect.append(bias_hat)
                pdis_glrt[i:next_i] = [data + bias_hat for data in pdis[i:next_i]]
                idx_sect.append(i)
    return MSE_sect, time_sect, grouppnt_sect, bias_sect, pdis_glrt, error_sect, idx_sect

def GLRT_phase(ptime, pdis, ctime, cdis):
    MSE_sect = []
    time_sect = []
    grouppnt_sect = []
    bias_sect = []
    pdis_glrt = pdis * 1
    error_sect = []
    min_point = 4
    wavelen = 0.1629247078010329
    #cdis_wrap =  [(i % wavelen) for i in cdis]
    for i,brkpnt in enumerate(brkpnts):
        if brkpnt == 1:
            try:
                next_i = brkpnts.index(1,i+1)
            except ValueError:
                next_i = len(brkpnts) - 1

            if next_i-i >= min_point:
                time_sect.append((ptime[next_i] + ptime[i] )/2)#(ptime[next_i] + ptime[i] )/2
                grouppnt_sect.append(next_i - i)
                #MLE
                cdis_itp_sect = []
                data_valid = True 
                for j in range(i,next_i):
                    #print(j,linear_itp(ptime[j],ctime,cdis))
                    a,b,flag = linear_itp(ptime[j],ctime,cdis)
                    cdis_itp_sect.append(b)
                    if not flag:
                        data_valid = False
                if data_valid:
                    bias_hat, MSE_temp = MLE_sect(pdis[i:next_i],cdis_itp_sect)
                else:
                    bias_hat = 0
                    MSE_temp = 1
                error_sect.append(data_valid)
                MSE_sect.append(MSE_temp)
                bias_sect.append(bias_hat)
                pdis_glrt[i:next_i] = [data + bias_hat for data in pdis[i:next_i]]
    return MSE_sect, time_sect, grouppnt_sect, bias_sect, pdis_glrt, error_sect


def brkpnt_det(ptime,pdis):
    brkpnts = []
    for i, pd in enumerate(pdis):
        if ptime[i]-ptime[i-1] > 0.2 or abs(pdis[i]-pdis[i-1]) > 0.06:
            brkpnts.append(1)
        else:
            brkpnts.append(0)
    if len(brkpnts) != 0:
        brkpnts[0] = 0
    return brkpnts

def get_antenna_dis(box_data, box_ids, antenna_loc, cv_label):
    cx = np.array([eval(i[-3]) for i in box_data[box_ids.index(cv_label)] if i[-2] != 'nan '])
    cy = np.array([eval(i[-2]) for i in box_data[box_ids.index(cv_label)] if i[-2] != 'nan '])
    cz = np.array([eval(i[-1]) for i in box_data[box_ids.index(cv_label)] if i[-2] != 'nan '])
    
    cdis_antenna_temp = []
    for i in range(len(cx)):
        cdis_antenna_temp.append( ((cx[i] + antenna_loc[0]) ** 2 + (cy[i] + antenna_loc[1]) ** 2 + (cz[i] + antenna_loc[2]) ** 2) ** 0.5 )
    cdis_antenna_output = cdis_antenna_temp * 1
    #print(cdis_antenna_output)
    return cdis_antenna_output

def str2timestamp(str_in,format):
    time = datetime.datetime.strptime(str_in,format)
    return datetime.datetime.timestamp(time)

def block_gen(tan):
    if tan < -1.2:
        b = 0
    elif tan < -0.8:
        b = 1
    elif tan < -0.4:
        b = 2
    elif tan < 0:
        b = 3
    elif tan < 0.4:
        b = 4
    elif tan < 0.8:
        b = 5
    elif tan < 1.2:
        b = 6
    else:
        b = 7
    return b
def block_genx(tan):
    if tan < -1.5:
        b = 0
    elif tan < -1:
        b = 1
    elif tan < -0.5:
        b = 2
    elif tan < 0:
        b = 3
    elif tan < 0.5:
        b = 4
    elif tan < 1:
        b = 5
    elif tan < 1.5:
        b = 6
    else:
        b = 7
    return b
def block_genz(tan):
    if tan < 1.2:
        b = 0
    elif tan < 2:
        b = 1
    elif tan < 2.8:
        b = 2
    elif tan < 3.6:
        b = 3
    elif tan < 4.4:
        b = 4
    elif tan < 5.2:
        b = 5
    elif tan < 6:
        b = 6
    else:
        b = 7
    return b

def poly_fit_return_parameter(x_seq, y_seq, x0, order = 2):
    x_og = x_seq * 1
    x_seq = np.array(x_seq) - x0
    y_seq = np.array(y_seq)
    m = []
    for i in range(order):
        a = x_seq ** (i)
        m.append(a)
    A = np.array(m).T
    b = y_seq.reshape(y_seq.shape[0],1)
    
    AA = A.T.dot(A) # A times A.transpose
    try:
        w=np.linalg.inv(AA).dot(A.T).dot(b)
    except:
        print(x_og)
        print(y_seq)
        print(A)
        print(AA)
        raise Exception("error")

    return w * 1

def poly_fit(x_seq, y_seq, x0, order = 3):
    x_og = x_seq * 1
    x_seq = np.array(x_seq) - x0
    y_seq = np.array(y_seq)
    m = []
    for i in range(order):
        a = x_seq ** (i)
        m.append(a)
    A = np.array(m).T
    b = y_seq.reshape(y_seq.shape[0],1)
    
    AA = A.T.dot(A) # A times A.transpose
    try:
        w=np.linalg.inv(AA).dot(A.T).dot(b)
    except:
        print(x_og)
        print(y_seq)
        print(A)
        print(AA)
        raise Exception("error")
    
    y_fit = A.dot(w)

    # fig, ax = plt.subplots()
    # ax.scatter(x_seq, y_fit)
    # ax.scatter(x_seq, y_seq)
    # plt.show()
    return y_fit * 1

def poly_fit_newx(x_seq, y_seq, x0, order = 3):
    x_og = x_seq * 1
    x_seq = np.array(x_seq) - x0
    y_seq = np.array(y_seq)
    m = []
    for i in range(order):
        a = x_seq ** (i)
        m.append(a)
    A = np.array(m).T
    b = y_seq.reshape(y_seq.shape[0],1)
    
    AA = A.T.dot(A) # A times A.transpose
    try:
        w=np.linalg.inv(AA).dot(A.T).dot(b)
    except:
        print(x_seq)
        print(y_seq)
        print(A)
        print(AA)
        raise Exception("error")
    # y_fit = A.dot(w)

    # fig, ax = plt.subplots()
    # ax.scatter(x_seq, y_fit)
    # ax.scatter(x_seq, y_seq)
    # print(A)
    A = np.zeros(order).T
    A[0] = 1
    y_fit = A.dot(w)
    # ax.scatter([0],[y_fit], s = 10 )
    # plt.show()
    return y_fit * 1

def holograph_poly_fit(x_seq, y_seq, x0, index, order = 3, new_x = False):
    sin_seq = [np.sin(i) for i in y_seq]
    cos_seq = [np.cos(i) for i in y_seq]
    order = np.minimum(len(set(x_seq)), order)
    if new_x:
        sin_fit = poly_fit_newx(x_seq, sin_seq , x0, order)
        cos_fit = poly_fit_newx(x_seq, cos_seq , x0, order)
    else:
        sin_fit = poly_fit(x_seq, sin_seq , x0, order)[index]
        cos_fit = poly_fit(x_seq, cos_seq , x0, order)[index]

    phi_tan = np.arctan(sin_fit/cos_fit)

    if cos_fit > 0:
        return phi_tan
    else:
        return phi_tan + np.pi

def Holograph_linear_itpl(cphi1, cphi2):
    sin1 = np.sin(cphi1)
    sin2 = np.sin(cphi2)
    cos1 = np.cos(cphi1)
    cos2 = np.cos(cphi2)
    sin_mean = (sin1 + sin2) / 2
    cos_mean = (cos1 + cos2) / 2

    phi_sin = np.arcsin(sin_mean)
    phi_cos = np.arccos(cos_mean)

    if phi_sin > 0:
        return phi_cos
    else:
        return -phi_cos 

def holograph(rad_list):
    if len(rad_list) == 0:
        return 0
    re = []
    im = []
    for rad in rad_list:
        re.append(math.cos(rad))
        im.append(math.sin(rad))
    re_mean = statistics.mean(re)
    im_mean = statistics.mean(im)
    if re_mean > 0:
        return np.arctan(im_mean/re_mean)
    elif im_mean > 0:
        return np.arctan(im_mean/re_mean) + np.pi
    else:
        return np.arctan(im_mean/re_mean) - np.pi

def holograph_addon(rad_list):
    if len(rad_list) == 0:
        return 0,0
    re = []
    im = []
    for rad in rad_list:
        re.append(math.cos(rad))
        im.append(math.sin(rad))
    re_sum = np.sum(re)
    im_sum = np.sum(im)
    amplitude = (re_sum ** 2 + im_sum ** 2) ** 0.5 / len(rad_list)
    if re_sum > 0:
        return np.arctan(im_sum/re_sum), amplitude
    elif im_sum > 0:
        return np.arctan(im_sum/re_sum) + np.pi, amplitude
    else:
        return np.arctan(im_sum/re_sum) - np.pi, amplitude

def holograph_re_addon(rad_list):
    if len(rad_list) == 0:
        return 0,0
    re = []
    im = []
    for rad in rad_list:
        re.append(math.cos(rad))
        im.append(math.sin(rad))
    re_sum = np.sum(re)
    im_sum = np.sum(im)
    amplitude = (re_sum ** 2 + im_sum ** 2) ** 0.5 / len(rad_list)
    if re_sum > 0:
        return np.arctan(im_sum/re_sum), re_sum / len(rad_list)
    elif im_sum > 0:
        return np.arctan(im_sum/re_sum) + np.pi, re_sum / len(rad_list)
    else:
        return np.arctan(im_sum/re_sum) - np.pi, re_sum / len(rad_list)

def Linear_fitting_2pnts(x1,x2,y1,y2,x_in):
    if x2 < x1:
        print('Error in Linear_fitting_2pnts') 
        return 0
    elif x2 == x1:
        alpha = 1
    else:
        alpha = (x_in - x1)/(x2 - x1)
    return alpha * y2 + (1-alpha) * y1 


def MSE(array):
    return np.mean(np.square(array)) ** 0.5

def MAE(array):
    return np.mean(np.abs(array)) 

def take_rssi_mean(array_db):
    #input must be list!
    array_db = np.array(array_db)
    #print(array_db/10)
    array = np.power(10,array_db/10 )
    #print(array)
    return np.log10(np.mean(array)) * 10

if __name__ == '__main__':

    print(take_rssi_mean([-45,-50,-51,-55]))