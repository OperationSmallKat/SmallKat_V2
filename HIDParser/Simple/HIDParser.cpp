 #include "HIDParser.h"
#define Timeout 10
PWMServo myServo[20];
int i = 0;
int n = 0;
byte buffer[64];
byte IMUBuffer[64];
int pos = 0;  
Adafruit_BNO055 bno = Adafruit_BNO055();

void HIDParser::Begin()
{
int  Servos[20]= {2, 3, 4, 5, 6, 7, 8, 9, 10, 29, 30, 23, 22, 21, 20, 14, 38, 37, 36, 35};
int  Min[20]= {1000,500,1000,500,500,500,500,1000,500,500,500,500,500,500,500,500,500,1000,500, 500};
int  Max[20]=
    {2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000};
	for (i = 0; i<(int)sizeof(Servos);i++)
	{
	myServo[i].attach(Servos[i], Min[i],Max[i]);
	delay(15);
	}
    bno.setExtCrystalUse(true);

}


void HIDParser::Server(void)
{
	n = RawHID.recv(buffer, Timeout);
  
  	if (n > 0) {
        if(buffer[0] == 0){//using byte 0 as a checksum for IMU data 0 for no IMU, 1 for IMU
            RawHID.send(buffer, Timeout);//Returns received array to sender
        }
        else{
            if(!bno.begin())
            {
                imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                IMUBuffer[0] = euler.x();
                IMUBuffer[1] = euler.y();
                IMUBuffer[2] = euler.z();
                //            float x_accel = (read8(BNO055_ACCEL_DATA_X_MSB_ADDR) << 8) | (read8(BNO055_ACCEL_DATA_X_LSB_ADDR));
                //            float y_accel = (read8(BNO055_ACCEL_DATA_Y_MSB_ADDR) << 8) | (read8(BNO055_ACCEL_DATA_Y_LSB_ADDR));
                //            float z_accel = (read8(BNO055_ACCEL_DATA_Z_MSB_ADDR) << 8) | (read8(BNO055_ACCEL_DATA_Z_LSB_ADDR));
            }
            else
            {
                IMUBuffer = 0;
            }
            RawHID.send(IMUBuffer, Timeout);//Returns array of IMU data to sender
        }
    for(i = 0;i<20;i++){
    	myServo[i].write(buffer[i+3]);
    }
	}
}
