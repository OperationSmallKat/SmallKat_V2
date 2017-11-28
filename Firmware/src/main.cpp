//
// Created by keionbis on 10/26/17.
//

#include "main.h"
//IntervalTimer RunEvery;

#define frequency 65536

int randnum = 0;
int i = 0;
int baudrate = 9600;

Servo TailUDServo;
Servo TailLRServo;
Servo BRFServo;
Servo BRSServo;
Servo BRMServo;
Servo FRFServo;
Servo FRMServo;
Servo FRSServo;
Servo HeadUDServo;
Servo BLFServo;
Servo BLMServo;
Servo BLSServo;
Servo HeadLRServo;
Servo FLMServo;
Servo FLFServo;
Servo FLSServo;

void setup() {

    Serial.begin(115200);
//Servo Initializations Begin

//    TailUDServo.attach(TailUDServo_PIN);
//    TailLRServo.attach(TailLRServo_PIN);
//    BLFServo.attach(BLFServo_PIN);
//    BLMServo.attach(BLMServo_PIN);
//    BLSServo.attach(BLSServo_PIN);
//    BRFServo.attach(BRFServo_PIN);
//    BRMServo.attach(BRMServo_PIN);
//    BRSServo.attach(BRSServo_PIN);
//    FRFServo.attach(FRFServo_PIN);
//    FRMServo.attach(FRMServo_PIN);
//    FRSServo.attach(FRSServo_PIN);
//    FLFServo.attach(FLFServo_PIN);
//    FLMServo.attach(FLMServo_PIN);
//    FLSServo.attach(FLSServo_PIN);
//    HeadLRServo.attach(HeadLRServo_PIN);
//    HeadUDServo.attach(HeadUDServo_PIN);

//Servo Initializations End




}
void loop() {
   // com.server();
    FRMServo.write(180);

}

void CycleLoop(){


    // TODO move this to its own file
    // TODO Grab all data over HID
    // TODO Update Servos



}