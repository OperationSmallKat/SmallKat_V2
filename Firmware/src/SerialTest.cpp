//
// Created by keionbis on 10/26/17.
//

#include "main.h"

void SerialSetup(){
    HWCOMMS.begin(2000000);

}
void SerialRun(){
    HWCOMMS.write("/r/n%d",1);
}

void TestSerial(){
    SerialSetup();
    while(1)
    {
        SerialRun();
    }
}