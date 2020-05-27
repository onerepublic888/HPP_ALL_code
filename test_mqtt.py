import paho.mqtt.client as mqtt  
from matplotlib.patches import Rectangle, PathPatch
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import sys, os, json, collections, time
from localization_algorithm_threading import costfun_method
from caliculate_error_threading import cal_error, cal_Anc_pos, remove_dis_err

host = '192.168.50.138'      #This computer
# host = '192.168.50.202'
port = 1883

topic = 'UWB' #receive data from rpi
topic_2 = 'Anc_switch'    #send switch to anchor0
topic_3 = 'localization_data'
topic_4 = 'power_reset'
topic_5 = 'mac'


tag_pos_data = collections.deque(maxlen=10000)
dis_array_data = collections.deque(maxlen=10000)
mqtt_data = collections.deque(maxlen=10000)
switch_ls = ['1o', '2o', '3o']
d01, d02, d03, d12, d13, d23 = [], [], [], [], [], []
global D0_ls
global D1_ls
global D1_ls
D0_ls, D1_ls, D2_ls = [],[],[]

global calibrate
global anchor_measure_dis
global localization
localization = False
anchor_measure_dis = False
calibrate = False


def on_message(client, userdata, msg):       # Get UWB data via MQTT from RPI4
    data = msg.payload.decode('utf-8')
    if  (msg.topic == topic ):
        recev = data.split(' ')
        # mac = recev[0]
        # UWB_ser_data = [int(recev[3],16),int(recev[4],16),int(recev[5],16),int(recev[6],16),int(recev[10][1])]
        # if int(recev[9][1]) != 7:
        UWB_ser_data = [int(recev[2],16),int(recev[3],16),int(recev[4],16),int(recev[5],16),int(recev[9][1])]
        dis_array_data.append(UWB_ser_data)

def get_anc_pos(UWB_data, calibrate, anchor_measure_dis, localization):  # parse UWB six distance data and send UWB switching Tag to Anchor command 
    dis0,dis1,dis2,dis3,tag = UWB_data[0], UWB_data[1], UWB_data[2], UWB_data[3], UWB_data[4]
    if(tag ==1 and len(d01) < m  and dis0!=0 and dis1 == 0 and dis2 == 0 and dis3==0):
        print('dis[01]: ', dis0)
        d01.append(dis0)

    elif(tag ==2 and len(d02) < m and dis0!=0 and dis1 == 0 and dis2 == 0 and dis3==0):
        print('dis[02]: ', dis0)
        d02.append(dis0)

    elif(tag ==3 and len(d03) < m and dis0!=0 and dis1 == 0 and dis2 == 0 and dis3==0):
        print('dis[03]: ', dis0)
        d03.append(dis0)

    elif(len(d01) >= m and len(d02) >= m and len(d03) >= m and dis0!= 0 and dis1 == 0 and dis2 == 0 and dis3==0 and len(D0_ls)<1):
        print('1A3T done: ', len(d01), len(d02), len(d03))
        dis_1state = np.vstack((np.vstack((np.array(d01), np.array(d02))), np.array(d03)))
        D0_ls.append(dis_1state)
        print('D0_ls: ',D0_ls)
        np.savez('UWB_dis_1state.npz', dis_1state = dis_1state)
        print('sent switch 1o')
        
    elif(len(D0_ls)>=1 and dis0!= 0 and dis1 == 0 and dis2 == 0 and dis3==0):
        client.publish(topic_2, json.dumps(switch_ls[0]))

    elif(tag ==2 and len(d12)< m and dis0!= 0 and dis1!= 0 and dis2 == 0 and dis3 == 0):
        print('dis[12]: ', dis1)
        d12.append(dis1)
        
    elif(tag ==3 and len(d13)< m and dis0!= 0 and dis1!= 0 and dis2 == 0 and dis3 == 0):
        print('dis[13]: ', dis1)
        d13.append(dis1)
        
    elif(len(d12) >= m and len(d13) >= m  and dis0!= 0 and dis1!= 0 and dis2 == 0 and dis3 == 0 and len(D1_ls)<1):
        print('2A2T done: ', len(d12), len(d13))
        dis_2state = np.vstack((np.array(d12), np.array(d13)))
        D1_ls.append(dis_2state)
        print('D1_ls: ',D1_ls)
        np.savez('UWB_dis_2state.npz', dis_2state = dis_2state)      
        print('sent switch 2o')

    elif(len(D1_ls)>=1 and dis0!= 0 and dis1!= 0 and dis2 == 0 and dis3 == 0):
        client.publish(topic_2, json.dumps(switch_ls[1]))
                                       
    elif(tag ==3 and len(d23)< m and dis0!= 0 and dis1!= 0  and dis2 != 0 and dis3 == 0):
        print('dis[23]: ', dis2)
        d23.append(dis2)
        
    elif(len(d23) >= m and dis0!= 0 and dis1!= 0  and dis2 != 0 and dis3 == 0 and len(D2_ls)<1 ):
        print('3A1T done: ', len(d23))
        D2_ls.append(d23)
        print('D2_ls: ',D2_ls)
        np.savez('UWB_dis_3state.npz', dis_3state = d23)
        print('sent switch 3o')

    elif(len(D2_ls)>=1 and dis0!= 0 and dis1!= 0  and dis2 != 0 and dis3 == 0 ):
        client.publish(topic_2, json.dumps(switch_ls[2]))      

    elif(dis0 != 0 and dis1 != 0 and dis2 != 0 and dis3 != 0 ): #If all UWB have switch to anchor 
        print('All HPP device have change to Anchor!')
        dis0_data, dis1_data, dis2_data = np.load('UWB_dis_1state.npz'), np.load('UWB_dis_2state.npz'), np.load('UWB_dis_3state.npz')
        D0, D1, D2 = dis0_data['dis_1state'], dis1_data['dis_2state'], dis2_data['dis_3state']
        dis_arr = np.vstack((np.vstack((D0, D1)), D2))
        dis_arr_mean = np.around(np.mean(dis_arr, axis=1), 2)
        print('dis_arr_mean:　', dis_arr_mean)
        if (calibrate == True ):
            gt_dis0 = int(input('Input dis01(mm): '))
            gt_dis1 = int(input('Input dis02(mm): '))
            gt_dis2 = int(input('Input dis03(mm): '))
            gt_dis3 = int(input('Input dis12(mm): '))
            gt_dis4 = int(input('Input dis13(mm): '))
            gt_dis5 = int(input('Input dis23(mm): '))
            gt_dis = np.array([gt_dis0, gt_dis1, gt_dis2, gt_dis3, gt_dis4, gt_dis5])
            err_array = cal_error(dis_arr_mean, gt_dis)
            print('UWB err_array: ', err_array)

            return True
            
        elif(anchor_measure_dis == True):
            print('Start auto measure anchor position !')                                
            err_data = np.load('UWB_cali_data.npz')
            U_er = err_data['error']
            dis_error = np.array([U_er[0]+U_er[1],U_er[0]+U_er[2],U_er[0]+U_er[3],U_er[1]+U_er[2],U_er[1]+U_er[3],U_er[2]+U_er[3]])
            # print('dis_error: ', dis_error)
            new_dis = dis_arr_mean - dis_error
            anc_pos = cal_Anc_pos(new_dis)
            np.savez('UWB_Anc_pos.npz', anc_pos = anc_pos)
            print('UWB position: ', anc_pos)
            
            return True
                
# def _main():


client = mqtt.Client()
client.connect(host, 1883, 60)
client.subscribe(topic)
client.subscribe(topic_2)
client.on_message = on_message
client.loop_start()
while True:
    choise = int(input('If Calibrate UWB--> input 0,If anchor distance measure input 1: , start localization 2: '))
    if (choise == 0 ):
        client.publish(topic_4, json.dumps('low'))
        m = int(input('Input measure times: '))
        time.sleep(1)
        client.publish(topic_4, json.dumps('high'))
        print('Reset')
        time.sleep(5)
        calibrate = True
        break

    elif(choise == 1):
        client.publish(topic_4, json.dumps('low'))
        m = int(input('Input measure times: '))
        time.sleep(1)
        client.publish(topic_4, json.dumps('high'))
        print('Reset')
        time.sleep(5)
        anchor_measure_dis = True
        break

    elif(choise == 2):
        print('start localization')
        localization = True
        break

    else:
        print('Wrong choise, choise again')
        continue


print('Clear dis array queue')
dis_array_data.clear()
time.sleep(1)
if localization == True:
    measure_done = True
    anc_pos_data = np.load('UWB_Anc_pos.npz')
    anc_pos = anc_pos_data['anc_pos'] 
else:
    measure_done = False
    
while True:
    if len(dis_array_data) > 0:
        UWB_data = dis_array_data.popleft()
        if not measure_done :
            # print(UWB_data)
            measure_done = get_anc_pos(UWB_data, calibrate, anchor_measure_dis, localization)
            anc_pos_data = np.load('UWB_Anc_pos.npz')
            anc_pos = anc_pos_data['anc_pos']
        else:
            # print(UWB_data)
            if(UWB_data[0] !=0 and UWB_data[1] !=0 and UWB_data[2] !=0 and UWB_data[3] !=0 ):
                dis_array = np.array([UWB_data[0], UWB_data[1], UWB_data[2], UWB_data[3]])
                # print(dis_array)
                Q = np.array((0.001, 0.001, 0.001))    # system noise (variance)
                R = np.array((0.0001, 0.0001, 0.01))              # measurement noise (variance)
                last_x, last_p = np.zeros_like(R), np.ones_like(Q) *1.
                

                tag_pos = costfun_method(dis_array, anc_pos)
                tag_pos_data.append(tag_pos)
                print(anc_pos)
                print(tag_pos)
            
    else:
        print('len(dis_array_data) == 0')

fig = plt.figure()
ax = fig.add_subplot(1,2,1, projection='3d')
ax2 = fig.add_subplot(2, 2, 2)
ax3 = fig.add_subplot(2, 2, 4)

def animate(i):
    if len(tag_pos_data) > 0:
        ax.clear()
        ax2.clear()
        ax3.clear()
        anc_pos_data = np.load('UWB_anc_pos.npz')
        anchor_positions = anc_pos_data['anc_pos']
        print('anchor_positions: ', anchor_positions)
        for i in range(len(anchor_positions)):
            ax.scatter(anchor_positions[i][0],anchor_positions[i][1],anchor_positions[i][2], color="k", s=100, label='Anchor '+str(i))
        tag_pos = tag_pos_data.popleft()
        print('Animate---> tag_pos: ',tag_pos)
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

ani = FuncAnimation(fig, animate, interval=200) 
print('Start animate function !')
plt.show()



        
# if __name__=='__main__':
#     _main()