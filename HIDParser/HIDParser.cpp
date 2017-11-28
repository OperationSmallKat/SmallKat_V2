#include "HIDParser.h"
#define Timeout 10
PWMServo myservo[MAX_SERVOS];
int i = 0;
int n = 0;
byte buffer[64];

void HIDParser::Begin( int *Servos)
{
	for (i = 0; i<(int)sizeof(Servos);i++)
	{
	myservo[i].attach(Servos[i]);
	//if(Serial)
	  //Serial.println("Attaching Servo %d on pin %d", i, Servos[i]);
	}
}


void HIDParser::Server(void)
{
	n = RawHID.recv(buffer, Timeout);
  RawHID.send(buffer, Timeout);
  	if (n > 0) {
    for(i = 0;i<MAX_SERVOS;i++){
    	myservo[i].write((int)buffer[i+4]);
      //if(Serial)
    	  //Serial.println((int)buffer[i+4]);
    }
	}
}
