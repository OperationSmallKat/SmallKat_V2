#ifndef HIDPARSER_H
#define HIDPARSER_H
//#define MAX_SERVOS 20

#include <Arduino.h>
#ifdef __cplusplus
	#include <SoftwareSerial.h>
	#include <PWMServo.h>
    #include <Wire.h>
    #include <Adafruit_Sensor.h>
    #include <Adafruit_BNO055.h>
    #include <utility/imumaths.h>
#endif


class HIDParser{
public:
	void Begin(void);
	void Server(void);


private:
#define BNO055_SAMPLERATE_DELAY_MS (100)
};

#endif
