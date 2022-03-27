// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Ballevator.h"

#include <constants.h>
#include <frc/smartdashboard/SmartDashboard.h>
using namespace Constants::ballelavator;

Ballevator::Ballevator()
    : elevator(ballelevator, rev::CANSparkMax::MotorType::kBrushless),
      intakesens{intakenum},
      elevsens{elevnum} {
  // Implementation of subsystem constructor goes here.
}

void Ballevator::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutBoolean(
      "Intake Sensor", intakesens.Get());  // for testing of the ball sensors
  frc::SmartDashboard::PutBoolean("Storage Sensor", elevsens.Get());
  frc::SmartDashboard::PutNumber("elevator power", elevpower);

  elevator.Set(elevpower);
}

int Ballevator::ballesense() {
  //   //sensor wrangling
  int intakecheck = intakesens.Get();
  int elevchek = elevsens.Get();
  int ballcount = 0;
  //   // set sensor 1 to a value of 0 or
  if (intakecheck == true &&
      elevchek == true) {  // check that there is nothing there
    ballcount = 0;         // count the ball amount to 0
    return 0;              // return stop power
  }
  if (intakecheck == false &&
      elevchek == true) {  // when a ball enters the intake
    ballcount = 1;         // tell drivers that they have one ball that he
    return 1;              // power set to slow to index the ball
  } else if (intakecheck == false &&
             elevchek == false) {  // sensor logic expressions;
    ballcount = 2;
    return 1;  // set power to index, tell drivers hopper is full
  } else {
    ballcount = 0;
    return 3;
  }
}

void Ballevator::startstop(int starter) {
  // Forward Reverse and Stop
  // ballloca == ball location, determining whether or not the power thould be
  // on off or reverse
  //  int balloca = ballesense();
  //  if (balloca == 1 & starter == 0)//if there is a ball, and no driver input,
  //  set to index power
  //    elevator.Set(.5);
  //  else
  if (starter == 1)  // when there is driver input set shoot power
    elevpower = .99;
  else
    elevpower = 0;
  // if (balloca == 0) //when there is not known inputs, set stop
  //   elevator.Set(0);
  // else
  //   if (starter == 3) //if the driver requests reverse, reverse the
  //   ballevator elevator.Set(-.5);
  //   //set reverse from button
}

void Ballevator::ballcounter() {}

void Ballevator::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
