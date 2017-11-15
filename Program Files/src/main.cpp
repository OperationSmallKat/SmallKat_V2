//
// Created by keionbis on 10/26/17.
//

#include "main.h"
IntervalTimer RunEvery;
#define frequency 65536
#define ServoToTune Servo2
int randnum = 0;
int i = 0;
void CycleLoop();
Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;
Servo Servo6;
Servo Servo7;
Servo Servo8;
Servo Servo9;
Servo Servo10;
Servo Servo11;
Servo Servo12;
Servo Servo13;
Servo Servo14;
Servo Servo15;
Servo Servo16;
Servo Servo17;
Servo Servo18;
Servo Servo19;
Servo Servo20;


void setup(){
//    HWCOMMS.begin(115200);
//    Serial.begin(115200);
    //RunEvery.begin(CycleLoop,1.5);
    Servo1.attach(Servo1_PIN);
    Servo2.attach(Servo2_PIN);
    Servo3.attach(Servo3_PIN);
    Servo4.attach(Servo4_PIN);
    Servo5.attach(Servo5_PIN);
    Servo6.attach(Servo6_PIN);
    Servo7.attach(Servo7_PIN);
    Servo8.attach(Servo8_PIN);
    Servo9.attach(Servo9_PIN);
    Servo10.attach(Servo10_PIN);
    Servo11.attach(Servo11_PIN);
    Servo12.attach(Servo12_PIN);
    Servo13.attach(Servo13_PIN);
    Servo14.attach(Servo14_PIN);
    Servo15.attach(Servo15_PIN);
    Servo16.attach(Servo16_PIN);
    Servo17.attach(Servo17_PIN);
    Servo18.attach(Servo18_PIN);
    Servo19.attach(Servo19_PIN);
    Servo20.attach(Servo20_PIN);




}
void loop() {
    for (i = 0; i < 255; i++){
        ServoToTune.write(i);
        delay(50);
    }
    for (i = 255; i < 0; i--){
        ServoToTune.write(i);
        delay(50);
    }
    delay(1000);
}

void CycleLoop(){


    // TODO move this to its own file
    // TODO Grab all data over HID
    // TODO Update Servos



}