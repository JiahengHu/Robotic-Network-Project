#!/bin/bash
#this program is supposed to be running on the resbarry pie
#it will call the GPIOtest.py first and then particle.cpp
#the two program will be running simultaneously
#the ip address can be subject to change

python GPIOtest.py &
./particle 137.146.183.19 51717 &
