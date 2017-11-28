#ifndef HIDPARSER_H
#define HIDPARSER_H
#define MAX_SERVOS 20

#include <Arduino.h>
#ifdef __cplusplus
	#include <SoftwareSerial.h>
	#include <PWMServo.h>
#endif


class HIDParser{
public:
	void Begin( int *Servos);
	void Server(void);


private:

};

#endif
