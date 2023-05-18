from RT_dataloader import *
import cv2
import scipy.interpolate

def AoA_to_angle_without_distortion_correct(tan_theta):
    #1080
    fx = 1057.4399
    u_0 = 960.99
    #720
    # fx = 528.6107177
    # u_0 = 630.80725
    u = fx * tan_theta + u_0
    return u

def reverse_AoA_to_angle_without_distortion_correct(u):
    #1080
    fx = 1057.4399
    u_0 = 960.99
    #720
    # fx = 528.6107177
    # u_0 = 630.80725
    tan_theta = (u - u_0) / fx
    theta = np.arctan(tan_theta)
    return theta
def grid_gen():
    grid = []
    for u in range(1920):
        grid.append(reverse_AoA_to_angle_without_distortion_correct(u))
    return np.array(grid) + np.pi/2

phase_calib = np.array([-1.19, 0.43, 0.88, -1.74])
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

def main():
    cap = cv2.VideoCapture(0)
    RFID_data_RT = RT_dataloader_6A(r'C:\Users\song0199\Desktop\RFID experiments data\2022.10.18\\test.csv',
                                 'E280 1160 6000 0209 F811 7224')
    grid = grid_gen()

    while(True):
        time_current = time.time()
        phase, ptime, rssi, empty = RFID_data_RT.get_new_RFIDlines()
        print(ptime,empty)
        phase_list, rssi_list = RFID_data_RT.get_data(time_current - 0.4)
        print(phase_list, rssi_list, time_current - 0.4)
        specturm = ULA_aoa_spectrum(np.array([phase_list]) + np.expand_dims(phase_calib, axis = 0) , interval = 0.1, Num_ant = 6, res = 200)

        x = np.linspace(0,np.pi,200)
        y_interp = scipy.interpolate.interp1d(x, specturm)
        sampled_specturm = y_interp(grid)

        ret, frame = cap.read()
        h,w,C = np.shape(frame)
        cut_frame = frame[:,:int(w/2),:]
        cut_frame = cv2.resize(cut_frame, (1920,1080))
        gray_aoa_specturm = np.expand_dims(sampled_specturm[0,:], axis = 0).repeat(1080, axis =0) * 255
        gray_aoa_specturm = np.expand_dims(gray_aoa_specturm.astype(np.uint8), axis = 2)

        heatmap = cv2.applyColorMap(gray_aoa_specturm, cv2.COLORMAP_HOT)
        # con_frame =  np.expand_dims(gray_aoa_specturm, axis) +
        superimposed_img = heatmap*0.1+cut_frame*0.9
        # print(superimposed_img)
        cv2.imshow('frame', superimposed_img.astype(np.uint8))
        # time.sleep(0.1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

main()
