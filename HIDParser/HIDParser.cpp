#include "HIDParser.h"
#define MAX_SERVOS 20
PWMServo myservo[MAX_SERVOS];
int i = 0;


void HIDParser::Begin( int *Servos)
{
	for (i = 0; i<sizeof(Servos);i++)
	{
	myservo[i].attach(Servos[i]);
	Serial.println("Attaching Servo %d on pin %d", i, Servos[i]);
	}
}


void HIDParser::Server(void)
{
	n = RawHID.recv(buffer, 10);
  	if (n > 0) {

    for(i = 0;i<64;i++){
    	myservo[i].Write((int)buffer[i]);
    	Serial.print((int)buffer[i]);
    }
}