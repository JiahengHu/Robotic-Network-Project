#! /usr/bin/python

import serial
from time import sleep


GPIOSerial = serial.Serial("/dev/ttyS0", baudrate=115200, timeout = None)


while(True):
    num=float(GPIOSerial.readline())
    if num==9999.99:
        sensor=[]
        for i in range(5):
            sensor.append(float(GPIOSerial.readline()))
        print sensor
        #three sensor readings and two velocity commands

        filename = "test.csv"

        fp = open(filename, 'w')
        for i in range(5):
            fp.write(str(sensor[i]))
            fp.write("\n")
        fp.close
        sleep(1)
