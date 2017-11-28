#include"HIDParser.h"
HIDParser MyHID;
int Servos[MAX_SERVOS] = {2,3,4,5,6,7,8,9,10};
void setup(){
	//Serial.begin(115200);
	MyHID.Begin(Servos);
}

void loop(){
	MyHID.Server();
	
}
