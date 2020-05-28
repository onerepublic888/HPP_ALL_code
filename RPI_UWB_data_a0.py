import serial, time, json, random
import numpy as np
import RPi.GPIO as GPIO
import sys, os, collections
import paho.mqtt.client as mqtt  
import paho.mqtt.publish as publish
from uuid import getnode

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)

#host = '192.168.1.34'
host = '192.168.50.202'
port_mqtt = 1883
client_id = 'HPP'
topic = 'UWB'     #receive data from rpi
topic_2 = 'Anc_switch'   #send switch to anchor0
topic_3 = 'local_data'
topic_4 = 'power_reset'

port = '/dev/ttyS0'
ser_lora = serial.Serial(port, 115200, timeout = 0.05)
switch_ls = ['1o', '2o', '3o']

publish_data = collections.deque(maxlen=1000)

def getmac():
    tmp = getnode()
    tmp = hex(tmp)
    mac = tmp[2:13]
    #print(mac)
    return mac

def on_message(client, userdata, msg):
    data = msg.payload.decode('utf-8')
    # print('Got topic:' + ' ' + msg.topic + ' ; payload: ' + data)
    if (msg.topic == topic_2):
        if(data.find('1o') >= 0):
            print(data)
            ser_lora.write(switch_ls[0].encode())
            
        elif(data.find('2o') >= 0):
            print(data)
            ser_lora.write(switch_ls[1].encode())
            
        elif(data.find('3o') >= 0):
            print(data)
            ser_lora.write(switch_ls[2].encode())
            
    #elif(msg.topic == topic_4):
     #   if(data.find('high') >= 0):
      #      print('turn on')
       #     GPIO.output(23, GPIO.HIGH)
        #elif(data.find('low') >= 0):
         #   print('turn off')
          #  GPIO.output(23, GPIO.LOW)
        
GPIO.output(23, GPIO.LOW)
time.sleep(1)
client = mqtt.Client()
client.connect(host, port_mqtt)
client.subscribe(topic_2)
#client.subscribe(topic_4)
GPIO.output(23, GPIO.HIGH)
client.on_message = on_message
mac = getmac()
time.sleep(2)
client.loop_start()
while True:
    rx = ser_lora.readline()
    ti = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    if(rx != ''):
        try:
            rx_d = rx.split('\r')#and int(rx_d[0][-3])!=7
            if rx_d[0]>0 and rx_d[0].find('mc')>=0:
                client.publish(topic, json.dumps(rx_d[0]))
                print(rx_d[0], ti)
            if rx_d[1]>0 and rx_d[1].find('mc')>=0:
                client.publish(topic, json.dumps(rx_d[1]))
                print(rx_d[1], ti)
            if rx_d[2]>0 and rx_d[2].find('mc')>=0:
                client.publish(topic, json.dumps(rx_d[2]))
                print(rx_d[2], ti)
            if rx_d[3]>0 and rx_d[3].find('mc')>=0:
                client.publish(topic, json.dumps(rx_d[3]))
                print(rx_d[3], ti)
            if rx_d[4]>0 and rx_d[4].find('mc')>=0:
                client.publish(topic, json.dumps(rx_d[4]))
                print(rx_d[4], ti)
            if rx_d[5]>0 and rx_d[5].find('mc')>=0:
                client.publish(topic, json.dumps(rx_d[5]))
                print(rx_d[5], ti)
                
            if rx_d[6]>0 and rx_d[6].find('mc')>=0:
                client.publish(topic, json.dumps(rx_d[6]))
                print(rx_d[6], ti)
         
            #print(mac,int((rx_slice[2]),16),int((rx_slice[3]),16),int((rx_slice[4]),16),int((rx_slice[5]),16))
        except ValueError:
            print('ValueError')
        except IndexError:
            pass
            #print('IndexError')


