#ifndef HIDPARSER_H
#define HIDPARSER_H

#include <Arduino.h>
#ifdef __cplusplus
	#include <SoftwareSerial.h>
	#include <PWMServo.h>
#endif

#ifdef __cplusplus


class HidParser{
public:
	void Begin(int *Servos);
	void Server(void);


private:

};

#endif
#endif