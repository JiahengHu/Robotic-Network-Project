#! /usr/bin/python
#This program reads in the input from the socket client, and send the information to megapie via GPIO

import serial
from socket import *

gpioSerial = serial.Serial("/dev/ttyS0", baudrate=115200)  # for gpio
# bluetoothSerial = serial.Serial("/dev/rfcomm0", baudrate=115200)  # for bluetooth

# initialize socket connection
serverPort = 12345
serverSocket = socket(AF_INET, SOCK_STREAM)
serverSocket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
serverSocket.bind(('', serverPort))
serverSocket.listen(1)
print('The server is ready to receive')

command = None

try:
    while command is None:
        connectionSocket, addr = serverSocket.accept()
        command = connectionSocket.recv(1024).decode()

        if command == 'q':  # q exits the program
            print('The server is closing')
            connectionSocket.close()  # close the socket
            exit()

        print('Info from client: ', command)
        gpioSerial.write(str(command))  # send to megapi
        command = None

except (KeyboardInterrupt, SystemExit):
    print('The server is closing')
    connectionSocket.close()  # close the socket
    exit()
