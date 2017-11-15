#include <Arduino.h>
#include<Servo.h>
#include<IntervalTimer.h>
#define HWCOMMS Serial1

#define Servo1_PIN 2 // tail
#define Servo2_PIN 3 // tail up/down
#define Servo3_PIN 4 // tail left right
#define Servo4_PIN 5 //back right foot
#define Servo5_PIN 6 // back right sholder
#define Servo6_PIN 7 //back right mid
#define Servo7_PIN 8 // front right foot
#define Servo8_PIN 9 // front right mid
#define Servo9_PIN 10 // front right sholder
#define Servo10_PIN 29// head up down
#define Servo11_PIN 30//back left foot
#define Servo12_PIN 23// back left mid
#define Servo13_PIN 22// back left sholder
#define Servo14_PIN 21//
#define Servo15_PIN 20//head left right
#define Servo16_PIN 14
#define Servo17_PIN 38//front left mid
#define Servo18_PIN 37//front left foot
#define Servo19_PIN 36// front left shoulder
#define Servo20_PIN 35

void TestSerial();