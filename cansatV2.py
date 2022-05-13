from curses import baudrate
from socket import timeout
import numpy as np
import random as rnd
import time
import pandas as pd

import serial
import string
import board
import busio
import digitalio as dgIO
import matplotlib.pyplot as plt
import PyLora as lora   #--!!!!!!!!!!!!!!!!!!!
import os

name = "kai001"

sys_counter = 1

if True: #------------system switch
    s = True
else:
    s = False

#data

#datalist = open("Data_list_1.csv" , "x")
df1 = pd.DataFrame()
df1.columns = ["thermalcam","Air pressure","GPS","airatm","gyrodata","temp","gases"] 


datasend = open("Data_send_1.txt" , "a")
datasend.write("here's the begining of datasend\n")
datasend.close()

def encrypt(s):
    l = ['5', '4', '6', '8', '2', '7', '1', '3', '0', '9']
    ciphertext = ""
    for i in s:
        if i in [',','.']:
            ciphertext +=i
        else:
            ciphertext += l[int(i)]
    return ciphertext
    #

#------------------------------------------------------------------------------------sensor_setup
#gps-setup
import pynmea2
import serial
#gps_uartport = "/dev/ttyAMA0" #ถ้าไม่ได้ลอง /dev/ttyS0

#gyro-setup
import board
import busio
import adafruit_mpu6050
gyro_i2Cport = busio.iC2(board.SCL , board.SDA , frequency = 400000)

#windspd-setup 
import RPi.GPIO as GPIO
FLOW_SENSOR_GPIO = 13 #-----------------port DigitalIN
GPIO.setmode(GPIO.BCM)
GPIO.setup(FLOW_SENSOR_GPIO, GPIO.IN, pull_up_down = GPIO.PUD_UP)
count = 0
start_counter = False
def countPulse(channel):
   global count
   if start_counter:
      count = count+1

from time import sleep
from SX127x.LoRa import *
from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD
BOARD.setup()
BOARD.reset()

while s:
    try:
        # global sys_counter
        if sys_counter>1000:
            s = False
        for t in range(3):
            #sensor-read
                # GPS - read 
            serialline = serial.Serial("/dev/ttyAMA0" , baudrate = "9600" , timeout = 1.0)
            gps_input = serialline.readline()
            gps_dataout = pynmea2.NMEAStreamReader()
            if gps_input[0:6] == "$GPGGA":
                gps_data = pynmea2.parse(gps_input)
                gps_time = gps_data.timestamp
                gps_lat = gps_data.latitude
                gps_lon = gps_data.longitude
                gps_alt = gps_data.altitude
            gps_NMEA = str(gps_data)
                #string data
            gps = f'{gps_lat}-{gps_lon}-{gps_alt}'  
                #gyro - read
            gyro_data = adafruit_mpu6050.MPU6050(gyro_i2Cport)
            accel = [float("{0:.2f}".format(i)) for i in list(gyro_data.acceleration)]
            gyro = [float("{0:.2f}".format(i)) for i in list(gyro_data.gyro)]
            temp_mpu = float("{0:.2f}"(gyro_data.temperature))
                #string data
            gyro = f'{gyro[0]}_{gyro[1]}_{gyro[2]}'
            accel = f'{accel[0]}_{accel[1]}_{accel[2]}'
            temperature1 = str(temp_mpu)

            #global count
            windspd_rpm = count * 60
            #string data
            windspd_mph = (windspd_rpm * 60 ) / 5280 #------------------รอ calibrate ที่รร.
            windspd_mps = windspd_mph / 2.2369
            windspd = f'{windspd_rpm}-{windspd_mph}-{windspd_mps}'
 
            #------------------------------------write into storage

            # 2.datalist
            '''datalist = open("Data_list_1.csv" , 'r')
            readdata_list =[]
            df1 = pd.read_csv("Data_list_1.csv")
            df2 = df1
            for n in range(len(df1.columns)):
                thecolumn = df1.loc[n].values().append(readdata_list[n-1])
                df2.loc[n] = thecolumn'''

            #windspd reset & read
            count = 0
            GPIO.add_event_detect(FLOW_SENSOR_GPIO, GPIO.FALLING, callback=countPulse)
            start_counter = True
            time.sleep(1)
            start_counter = False
            #เก็บค่าเค้าท์เอาไว้รอบหน้า

        #-------------sendpackage2
        #packet2_data = [gps,windspd,accel,gyro_xyz,gas]
        packet_message = f'{gps_time},{gps},{windspd},{gyro},{temperature1}'
        packet_encripted = encrypt(packet_message)
        lora.sendpacket(packet_encripted)

        # 3.datasend
        datasend = open("Data_send_1.txt" , "a")
        datasend.writelines(f'Timestamp:{Time} / message No:{sys_counter}\n')
        datasend.writelines(f'packet= {packet_message} -> {packet_encripted} \n]')
        datasend.writelines("------------------------------------------------------------------------------\n")
        datasend.close()
        sys_counter +=1
    except:
        os.system("sudo reboot")

#เข้า@idle หลังจากขึ้นบิน 30 นาที
#ส่งข้อมูลจีพีเอสตำแหน่งอีกสามชั่วโมง
while True:
    try:
        serialline = serial.Serial("/dev/ttyAMA0" , baudrate = "9600" , timeout = 1.0)
        gps_input = serialline.readline()
        gps_dataout = pynmea2.NMEAStreamReader()
        if gps_input[0:6] == "$GPGLL":
            gps_data = pynmea2.parse(gps_input)
            gps_time = gps_data.timestamp
            gps_lat = gps_data.latitude
            gps_lon = gps_data.longitude
        lora.send_packet(f'{gps_time},{gps_lat},{gps_lon}')
        time.sleep(15)
    except:
        os.system("sudo reboot")
#----------------------------------------------------------------------------------------------------------------------------#