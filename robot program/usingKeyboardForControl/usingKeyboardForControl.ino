/**
 * Using MegaPi to control the robotic arm through Bluetooth communication between Mega and RPi3 to get input
 * w - forward
 * a - left
 * s - Backward
 * d - Right
 * 
 * space or x - stop
 * 
 * Serial2 for gpio
 * Serial3 for bluetooth
 * 
 * Then, send the velocity and sensor reading to the RPi3 via GPIO
 * The velocity was measured by running the bartender robot on the corridor of Davis with motor speed = 60
 * 0.0876m/s
 * 0.7519rad/s
 * 
 * Here is the velocity with motor speed = 100, in case it's needed
 * 0.193m/s
 * 1.784rad/s
 * 
 * Arthur: Jiaheng Hu
 *          Arthur Mukumbi
 * 
 */

#include "MeMegaPi.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

MeMegaPiDCMotor rightMotor(PORT2B);
MeMegaPiDCMotor leftMotor(PORT1B);
MeUltrasonicSensor RSensor(PORT_7);
//the right sonar
MeUltrasonicSensor LSensor(PORT_6);
//the left sonar
MeUltrasonicSensor FSensor(PORT_8);
//the front sonar

long count = 0;
int16_t motorSpeed = 60;
const int Nsensor =3;//the number of sensors
float sensor[Nsensor] = {0}; 
float velocity[2] = {0};

void setup() {
  Serial2.begin(115200);
  
}

void loop() {
  // listen for the data
  if (Serial2.available() > 0) {
    // read data from serial port
    char command = Serial2.read();  // read the received character from rpi
    switch (command){
      case 'd':
        TurnRight();
        break;
      case 'w':
        Forward();
        break;
      case 'a':
        TurnLeft();
        break;
      case 's':
        Backwards();
        break;
      case 'x':
        Stop();
        break;
    }
  }
  
  //send the information out
  sensor[0] = RSensor.distanceCm()*10;
  sensor[1] = FSensor.distanceCm()*10;
  sensor[2] = LSensor.distanceCm()*10;
  if (Serial2.availableForWrite() >= 20) {
    for(int i=0; i < Nsensor; i++){
        Serial2.print(sensor[i]);
        Serial2.print('\n');
    }
    Serial2.print(velocity[0]);
    Serial2.print('\n');
    //this is the velocity of the robot(m/s)
    
    Serial2.print(velocity[1]);
    Serial2.print('\n');
    //this is the angular velocity of the robot(rad/s)

    Serial2.print(9999.99);
    Serial2.print('\n');
    //A signal that shows that a new group begins
  }
  
  delay(1000);
  
}


void Forward(void){
  leftMotor.run(motorSpeed);
  rightMotor.run(-motorSpeed);
  velocity[0]=0.0876;
  velocity[1]=0;
}

void Backwards(void){
  leftMotor.run(-motorSpeed);
  rightMotor.run(motorSpeed);  
  velocity[0]=-0.0876;
  velocity[1]=0;
}

void TurnLeft(void){
  leftMotor.run(-motorSpeed); 
  rightMotor.run(-motorSpeed);
  velocity[0]=0;
  velocity[1]=0.7519;
}

void TurnRight(void){
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);   
  velocity[0]=0;
  velocity[1]=-0.7519;
}

void Stop(void){
  leftMotor.run(0);
  rightMotor.run(0);
  velocity[0]=0;
  velocity[1]=0;
}


