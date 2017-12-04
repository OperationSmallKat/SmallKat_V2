 #include "HIDParser.h"
#define Timeout 10
PWMServo myServo[20];
int i = 0;
int n = 0;
byte buffer[64];
int pos = 0;  

void HIDParser::Begin()
{
int  Servos[20]= {2, 3, 4, 5, 6, 7, 8, 9, 10, 29, 30, 23, 22, 21, 20, 14, 38, 37, 36, 35};
int  Min[20]= {1000,500,1000,500,500,500,500,1000,500,500,500,500,500,500,500,500,500,1000,500, 500};
int  Max[20]= {2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000};
	for (i = 0; i<(int)sizeof(Servos);i++)
	{
	myServo[i].attach(Servos[i], Min[i],Max[i]);
	delay(15);
	}
}


void HIDParser::Server(void)
{
	n = RawHID.recv(buffer, Timeout);
  
  	if (n > 0) {
      RawHID.send(buffer, Timeout);
    for(i = 0;i<20;i++){
    	myServo[i].write(buffer[i+3]);
    	  //Serial.println(buffer[i]);
    }
	}
}
