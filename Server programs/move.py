# This program is used together with the node js program
# It reads in the user's selection on the website and send it to the 
# socket server on the risberry pie
# Arthur: Jiaheng Hu

import socket
import sys
import json


serverName = '137.146.188.249'  # ip address of raspberry pi
serverPort = 12345


clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clientSocket.connect((serverName, serverPort))

command = sys.stdin.readlines()
command = json.loads(command[0])
command = command['c']
clientSocket.send(command.encode())
clientSocket.close()
