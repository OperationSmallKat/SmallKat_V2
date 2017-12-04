#include"HIDParser.h"
HIDParser * MyHID;
#define Joint1Min 800
#define Joint2Min 1000
#define Joint3Min 1500
#define Joint1Max 1600
#define Joint2Max 1800
#define Joint3Max 2000
//int Servos[MAX_SERVOS]= {2, 3, 4, 5, 6, 7, 8, 9, 10, 29, 30, 23 };
//int Max[MAX_SERVOS] = {Joint1Min,Joint2Min,Joint3Min,Joint1Min,Joint2Min,Joint3Min,Joint1Min,Joint2Min,Joint3Min,Joint1Min,Joint2Min,Joint3Min};
//int Min[MAX_SERVOS] = {Joint1Max,Joint2Max,Joint3Max,Joint1Max,Joint2Max,Joint3Max,Joint1Max,Joint2Max,Joint3Max,Joint1Max,Joint2Max,Joint3Max};
void setup(){
	Serial.begin(115200);
  Serial.println("Start");
  MyHID = new HIDParser();
  MyHID->Begin();
 
	
}

void loop(){
	MyHID->Server();
	
}
