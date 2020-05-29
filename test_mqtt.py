import paho.mqtt.client as mqtt  
from matplotlib.patches import Rectangle, PathPatch
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import sys, os, json, collections, time, csv
from datetime import datetime
from localization_algorithm_threading import costfun_method
from caliculate_error_threading import cal_error, cal_Anc_pos, remove_dis_err
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise, Saver
from filterpy.kalman import KalmanFilter
import threading
import json
# from GPS_Threading_new2 import GPS_Rx

host = '192.168.50.138'      #This computer
# host = '192.168.50.202'
port = 1883

topic = 'UWB' #receive data from rpi
topic_2 = 'Anc_switch'    #send switch to anchor0
topic_3 = 'localization_data'
topic_4 = 'power_reset'
topic_5 = 'mac'
string_time = datetime.now().strftime("%H_%M_%S")
calibrate_data_filename = 'calibrate_data_' + string_time +'.txt'
offline_data_filename = 'offline_data_' + string_time +'.txt'

tag_pos_data = collections.deque(maxlen=10)
dis_array_data = collections.deque(maxlen=10)
mqtt_data = collections.deque(maxlen=10)
switch_ls = ['1o', '2o', '3o']
d01, d02, d03, d12, d13, d23 = [], [], [], [], [], []
D0_ls, D1_ls, D2_ls = [],[],[]

localization = False
calibration_dist = None

new_data = threading.Event()

def on_message(client, userdata, msg):       # Get UWB data via MQTT from RPI4
    data = msg.payload.decode('utf-8')
    if  (msg.topic == topic ):
        recev = data.split(' ')
        # mac = recev[0]
        # UWB_ser_data = [int(recev[3],16),int(recev[4],16),int(recev[5],16),int(recev[6],16),int(recev[10][1])]
        # if int(recev[9][1]) != 7:
        UWB_ser_data = [int(recev[2],16),int(recev[3],16),int(recev[4],16),int(recev[5],16),int(recev[9][1])]
        dis_array_data.append(UWB_ser_data)
        new_data.set()


client = mqtt.Client()
client.connect(host, 1883, 60)
client.subscribe(topic)
client.subscribe(topic_2)
client.on_message = on_message
client.loop_start()
# GPS_port = GPS_Rx()
# GPS_port.start()

while True:
    choise = int(input('If Calibrate UWB--> input 0,If anchor distance measure input 1: , start localization 2: '))
    
    if (choise == 0 ):
        m = int(input('Input measure times: '))
        anchor_offset_high = round(float(input('Input anchor height(meters): ')), 1)

        gt_dis0 = int(input('Input dis01(mm): '))
        gt_dis1 = int(input('Input dis02(mm): '))
        gt_dis2 = int(input('Input dis03(mm): '))
        gt_dis3 = int(input('Input dis12(mm): '))
        gt_dis4 = int(input('Input dis13(mm): '))
        gt_dis5 = int(input('Input dis23(mm): '))
        gt_dis = np.array([gt_dis0, gt_dis1, gt_dis2, gt_dis3, gt_dis4, gt_dis5])

        client.publish(topic_4, json.dumps('low'))
        time.sleep(1)
        client.publish(topic_4, json.dumps('high'))
        print('Reset')
        time.sleep(5)
        measure_done = False
        calibration_dist = gt_dis
        break

    elif(choise == 1):
        m = int(input('Input measure times: '))
        anchor_offset_high = round(float(input('Input anchor height(meters): ')), 1)
        client.publish(topic_4, json.dumps('low'))
        time.sleep(1)
        client.publish(topic_4, json.dumps('high'))
        print('Reset')
        time.sleep(5)
        measure_done = False
        break

    elif(choise == 2):
        print('start localization')
        localization = True
        measure_done = True
        anc_pos_data = np.load('UWB_Anc_pos.npz')
        anc_pos = anc_pos_data['anc_pos']
        break

    else:
        print('Wrong choise, choise again')
        continue

time.sleep(1)


print('Clear dis array queue')
dis_array_data.clear()
new_data.clear()

# parse UWB six distance data and send UWB switching Tag to Anchor command
while not measure_done:
    if not new_data.wait(timeout=30.):
        print('new_data.wait(timeout=30.)')
        exit(0)
    try:
        UWB_data = dis_array_data.popleft()
    except IndexError:
        new_data.clear()
        continue

    dis0, dis1, dis2, dis3, tag = UWB_data[0], UWB_data[1], UWB_data[2], UWB_data[3], UWB_data[4]
    if( dis0!=0 and dis1 == 0 and dis2 == 0 and dis3 == 0):
        if(tag == 1 and len(d01) < m):
            print('dis[01]: ', dis0)
            d01.append(dis0)
        elif(tag == 2 and len(d02) < m):
            print('dis[02]: ', dis0)
            d02.append(dis0)
        elif(tag == 3 and len(d03) < m):
            print('dis[03]: ', dis0)
            d03.append(dis0)
        elif(len(d01) >= m and len(d02) >= m and len(d03) >= m  and len(D0_ls)<1):
            print('1A3T done: ', len(d01), len(d02), len(d03))
            dis_1state = np.vstack((np.vstack((np.array(d01), np.array(d02))), np.array(d03)))
            D0_ls.append(dis_1state)
            print('D0_ls: ',D0_ls)
            np.savez('UWB_dis_1state.npz', dis_1state = dis_1state)
        elif(len(D0_ls) >= 1):
            client.publish(topic_2, json.dumps(switch_ls[0]))
            print('sent switch 1o')
    elif(dis0!= 0 and dis1!= 0 and dis2 == 0 and dis3 == 0):
        if(tag ==2 and len(d12)< m ):
            print('dis[12]: ', dis1)
            d12.append(dis1)
        elif(tag ==3 and len(d13)< m ):
            print('dis[13]: ', dis1)
            d13.append(dis1)
        elif(len(d12) >= m and len(d13) >= m and len(D1_ls)<1):
            print('2A2T done: ', len(d12), len(d13))
            dis_2state = np.vstack((np.array(d12), np.array(d13)))
            D1_ls.append(dis_2state)
            print('D1_ls: ',D1_ls)
            np.savez('UWB_dis_2state.npz', dis_2state = dis_2state)
        elif(len(D1_ls)>=1):
            client.publish(topic_2, json.dumps(switch_ls[1]))
            print('sent switch 2o')            
    elif(dis0!= 0 and dis1!= 0  and dis2 != 0 and dis3 == 0):
        if(tag ==3 and len(d23)< m ):
            print('dis[23]: ', dis2)
            d23.append(dis2)
        elif(len(d23) >= m and len(D2_ls)<1 ):
            print('3A1T done: ', len(d23))
            D2_ls.append(d23)
            print('D2_ls: ',D2_ls)
            np.savez('UWB_dis_3state.npz', dis_3state = d23)            
        elif(len(D2_ls)>=1):
            client.publish(topic_2, json.dumps(switch_ls[2]))  
            print('sent switch 3o')
                                        
    elif(dis0 != 0 and dis1 != 0 and dis2 != 0 and dis3 != 0 ): #If all UWB have switch to anchor 
        print('All HPP device have change to Anchor!')
        dis0_data, dis1_data, dis2_data = np.load('UWB_dis_1state.npz'), np.load('UWB_dis_2state.npz'), np.load('UWB_dis_3state.npz')
        D0, D1, D2 = dis0_data['dis_1state'], dis1_data['dis_2state'], dis2_data['dis_3state']
        dis_arr = np.vstack((np.vstack((D0, D1)), D2))
        dis_arr_mean = np.around(np.mean(dis_arr, axis=1), 2)
        print('dis_arr_mean:　', dis_arr_mean)
        measure_done = True


if calibration_dist is not None:
    err_array = cal_error(dis_arr_mean, calibration_dist)
    print('UWB err_array: ', err_array)
    np.savez('UWB_cali_data.npz', dis_arr_mean = dis_arr_mean,  calibration_dist = calibration_dist, error = err_array)
    
    with open(calibrate_data_filename,'a') as fout:
        json.dump({'time': string_time, 'dis_arr_mean': dis_arr_mean.tolist(), 'calibration_dist': calibration_dist.tolist(), 'err_array': err_array.tolist()}, fout)

    print('calibration done.')
    exit(0)    

if not localization:
    print('Start auto measure anchor position !')                                
    err_data = np.load('UWB_cali_data.npz')
    U_er = err_data['error']
    dis_error = np.array([U_er[0]+U_er[1],U_er[0]+U_er[2],U_er[0]+U_er[3],U_er[1]+U_er[2],U_er[1]+U_er[3],U_er[2]+U_er[3]])
    print('dis_error: ', dis_error)
    new_dis = np.around((dis_arr_mean - dis_error)/1000, 2)
    for _ in range(10):
        print('new_dis: ', new_dis)
    anc_pos = cal_Anc_pos(new_dis, anchor_offset_high)
    np.savez('UWB_Anc_pos.npz', anc_pos = anc_pos)
    for _ in range(10):
        print('UWB position: ', anc_pos)
    filename = str(string_time)
    with open('anchor_measure_dis_data_' + filename +'.txt', 'a') as fout:
        json.dump({'time': string_time, 'new_dis': new_dis.tolist(), 'anc_pos': anc_pos}, fout)

    
np.set_printoptions(suppress=True) 
f = KalmanFilter (dim_x=2, dim_z=1)
f.F = np.array([[1.,1.],[0.,1.]])
f.H = np.array([[1.,0.]])              
f.P = np.array([[1.,    0.], [   0., 1.] ])
f.R = np.array([[0.1**2]])    # uwb dis std **2
saver_kf = Saver(f)
last_data_time = time.time()
#-------------------- animation parameter ---------------------
fig = plt.figure()
ax = fig.add_subplot(1,2,1, projection='3d')
ax2 = fig.add_subplot(2, 2, 2)
ax3 = fig.add_subplot(2, 2, 4)

err_data = np.load('UWB_cali_data.npz')
U_er = err_data['error']
anc_dis_error = np.array([U_er[0], U_er[1], U_er[2], U_er[3]])

def animate(i):
    try:
        UWB_data = dis_array_data.popleft()
    except IndexError:
        return

    if(UWB_data[0] !=0 and UWB_data[1] !=0 and UWB_data[2] !=0 and UWB_data[3] !=0 ):
        print('------------------------------------')
        dis_array = np.asarray([UWB_data[0], UWB_data[1], UWB_data[2], UWB_data[3]]) / 1000
        global anc_dis_error
        dis_er = np.around((np.asarray(anc_dis_error) / 1000), 2) + 0.3
        # print('anc_dis_error: ', dis_er )
        calibrate_dis_array = dis_array - dis_er
        origin_tag_pos = costfun_method(calibrate_dis_array, anc_pos)
        origin_tag_pos = np.around(origin_tag_pos, 2)

        global last_data_time
        # print('last_data_time: ', last_data_time)
        now_time = time.time()
        dt = now_time - last_data_time
        last_data_time = now_time

        # dt = 0.1
        f.x = np.array([origin_tag_pos[2], 0], dtype=np.float)    #  position,velocity
        f.F = np.array([[1, dt], [0, 1]], dtype=np.float)
        f.predict()
        # f.Q = Q_discrete_white_noise(2, dt=dt,var=2e-5, block_size=2)
        f.Q = Q_discrete_white_noise(dim=2, dt=1, var=1e-4)   
        f.update(z = origin_tag_pos[2])
        f.predict(F = np.array([[1, dt], [0, 1]]))
        saver_kf.save()
        z_predict = np.array(saver_kf.x)[-1,0]

        origin_tag_pos[2] = z_predict
        predict_pos = np.around(origin_tag_pos, 2)

        with open(offline_data_filename, 'a') as fout:
            json.dump({'time': string_time, 'predict_pos': predict_pos.tolist(), 'dis_array': calibrate_dis_array.tolist()}, fout)       

        # print('origin_tag_pos: ', origin_tag_pos)
        # print('predict_tag_pos: ', predict_tag_pos)

        ax.clear()
        ax2.clear()
        ax3.clear()
        tag_pos = predict_pos
        print_time = datetime.now().strftime("%H:%M:%S")
        print('Animate---> Tag_pos: ', tag_pos, print_time)
        print('Anchor_positions: ', anc_pos)
        for i in range(len(anc_pos)):
            ax.scatter(anc_pos[i][0],anc_pos[i][1],anc_pos[i][2], color="k", s=100, label='Anchor %d' % i)
        
        x,y,z = [tag_pos[0]], [tag_pos[1]], [tag_pos[2]]
        # ax.plot(x, y, z, 'b', label='Tag')
        ax.scatter(x, y, z, 'b',s=100, label='Tag')
        ax.set_xlim3d([-1.0, 5.0])
        ax.set_xlabel('x [m]', fontsize=16)  
        ax.set_ylim3d([-1.0, 5.0])

        ax.set_ylabel('y [m]', fontsize=16)  
        ax.set_zlim3d([0.0, 10.0])
        ax.set_zlabel('z [m]', fontsize=16)
        ax.legend()
        # ax.legend(labels = ['Z with KF', 'Z raw data'], loc = 'lower right' )
        ax2.plot(y,z, linestyle="", marker="o")
        ax2.set_xlabel('y [m]')
        ax2.set_ylabel('z [m]')
        ax2.set_xlim(-5, 5)
        ax2.set_ylim(0, 5)
        ax2.grid(True) 
        ax3.plot(x,z,  linestyle="", marker="o")
        ax3.set_xlabel('x [m]' )
        ax3.set_ylabel('z [m]')
        ax3.set_xlim(-5, 5)
        ax3.set_ylim(0, 5)
        ax3.grid(True) 


ani = FuncAnimation(fig, animate, interval=100) 
plt.show()