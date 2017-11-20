//
// Created by keionbis on 11/14/17.
//

#include "main.h"
#define PWMMax 255
#define DegreeMax 270
int DegreesToPWM(float degree);
void MoveServosToPos(int Leg1Link1, int Leg1Link2, int Leg1Link3, int Leg2Link1, int Leg2Link2, int Leg2Link3, int Leg3Link1, int Leg3Link2, int Leg3Link3, int Leg4Link1, int Leg4Link2, int Leg4Link3 );
int PWMval = 0;

void StandUp(){
    int Link1 = 0, Link2 = 0, Link3 = 0;

    Link1 = DegreesToPWM(130);
    Link2 = DegreesToPWM(90);
    Link3 = DegreesToPWM(90);

    //MoveServosToPos(Link1, Link2, Link3,Link1, Link2, Link3,Link1, Link2, Link3, Link1, Link2, Link3);

}

int DegreesToPWM(float degree){
    PWMval = PWMMax*degree/DegreeMax;
    return (PWMval);
}

void MoveServosToPos(int Leg1Link1, int Leg1Link2, int Leg1Link3, int Leg2Link1, int Leg2Link2, int Leg2Link3, int Leg3Link1, int Leg3Link2, int Leg3Link3, int Leg4Link1, int Leg4Link2, int Leg4Link3 )
{

}