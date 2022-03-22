// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Ballevator.h"
#include <constants.h>
using namespace Constants::ballelavator;


Ballevator::Ballevator() : elevator (ballelevator, rev::CANSparkMax::MotorType::kBrushless){
  // Implementation of subsystem constructor goes here.
}

void Ballevator::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void Ballevator::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
