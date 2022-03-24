// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Ballevator.h"
#include <constants.h>
#include <frc/smartdashboard/SmartDashboard.h>
using namespace Constants::ballelavator;


Ballevator::Ballevator() : elevator (ballelevator, rev::CANSparkMax::MotorType::kBrushless){
  // Implementation of subsystem constructor goes here.
}

void Ballevator::Periodic() {
  // Implementation of subsystem periodic method goes here.
    frc::SmartDashboard::PutBoolean("Intake Sensor", intakesens.Get());   // for testing of the ball sensors
    frc::SmartDashboard::PutBoolean("Storage Sensor", elevsens.Get());

}


int Ballevator::ballesense(){
//   //sensor wrangling
//   // set sensor 1 to a value of 0 or  
//   if (1==1)//sensor logic expression;
//     return 0;
//   else
//   if (1<0) //sensor logic expressions;
//     return 1;
//   else  

}

void Ballevator::startstop() {
  //Forward Reverse and Stop
  int balloca = ballesense();
  if (balloca == -1)// index power
    elevator.Set(.5);
  else
  if (balloca == 1) //set shoot power
    elevator.Set(.99);
  else
  if (balloca == 0) //set stop
    elevator.Set(0);
  else 
    elevator.Set(-.5);
    //set reverse from button

}

void Ballevator::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
