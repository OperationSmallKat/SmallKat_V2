//
// Created by keionbis on 11/16/17.
//

#ifndef MAIN_H
#define MAIN_H



#include <Arduino.h>
#include<Servo.h>
#include<IntervalTimer.h>
//#include "BowlerCom/src/BowlerCom.h"
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "MotorCalibration.h"


#define HWCOMMS Serial1



/*ServoPins
 *
 * 2  unused
 * 3 Tail Up/ Down
 * 4 Tail Left/Right
 * 5 Back Right Foot
 * 6 Back Right Shoulder
 * 7 Back Right Mid
 * 8 Front Right Foot
 * 9 Front Right Mid
 * 10 Front Right Shoulder
 * 29 Head Up Down
 * 30 Back Left Foot
 * 23 Back Left Mid
 * 22 Back Left Shoulder
 * 21 Unused
 * 23 Back Left Mid
 * 20 Head Left Right
 * 14 Unused
 * 38 Front Left Mid
 * 37 Front Left Foot
 * 36 Front Left Shoulder
 * 35 Unused
 *
 */
#define TailUDServo_PIN 29
#define TailLRServo_PIN 20
#define BRFServo_PIN 5
#define BRSServo_PIN 6
#define BRMServo_PIN 7
#define FRFServo_PIN 8
#define FRMServo_PIN 9
#define FRSServo_PIN 10
#define HeadUDServo_PIN 3
#define BLFServo_PIN 30
#define BLMServo_PIN 23
#define BLSServo_PIN 22
#define HeadLRServo_PIN 4
#define FLMServo_PIN 38
#define FLFServo_PIN 30
#define FLSServo_PIN 36


#define ServoToTune Servo2








void TestSerial();
long map(long x, long in_min, long in_max, long out_min, long out_max);

#endif //PROGRAM_MOTORCALIBRATION_H