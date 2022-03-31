/*
This file will contain the implementations for the functions in autoaim.cpp 
(will automatically aim and shoot the ball for the first 15 seconds of the match )

*/

//Inclusion of other files to run the functions in this file

#include "commands/autoaim.h"


autoaim::autoaim( Turret &t, Ballevator &b)
    : turret(t), ballevator(b){
      AddRequirements(&t);
    }


//initialiser function (to start up the turret wheels (motors related to turret))

void autoaim::Initialize() {
// starts up the ashooting wheels (gets it going)
    turret.Fire(true);

}

//execute function (will take place during autonomous 15 seconds)
//runs more than 50 times per second

void autoaim::Execute() {

  turret.Fire(true);

//goes to function that automates turret rotation and angle (autoaim turret)
turret.AutoTarget(true, true);

//should shoot the ball when it is ready to do so

if(turret.ReadyToFire()){

  ballevator.SetSpeed(BALLEVATOR_SPEED_FIRE);

}
else{

    
  ballevator.SetSpeed(BALLEVATOR_SPEED_READY);
}



}


//will take place when the robot ends its automation period
void autoaim::End(bool){

//  stops the ashooting wheels (makes them turn off)
    turret.Fire(false);

}

