import serial, time, json, random
import numpy as np
import sys, os, collections
import paho.mqtt.client as mqtt  
import paho.mqtt.publish as publish
from uuid import getnode
import RPi.GPIO as GPIO


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
topic_5 = 'mac'

#port = '/dev/ttyS0'
#ser_lora = serial.Serial(port, 115200, timeout = 0.001)
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
    if(msg.topic == topic_4):
        if(data.find('high') >= 0):
            print('turn on')
            GPIO.output(23, GPIO.HIGH)
        elif(data.find('low') >= 0):
            print('turn off')
            GPIO.output(23, GPIO.LOW)


def _main():
    client = mqtt.Client()
    client.connect(host, port_mqtt)
    client.subscribe(topic_4)
    client.on_message = on_message
    mac = getmac()
    client.loop_start()
    while True:
        payload = [mac + ' ' + 'A3' + ';']
        print(payload)
        #client.publish(topic_5, json.dumps(payload))
        time.sleep(1)

if __name__ == "__main__":
    _main()