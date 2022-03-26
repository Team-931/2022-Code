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
      elevsens{elevnum} {}

void Ballevator::Periodic() { elevator.Set(elevpower); }

void Ballevator::SetSpeed(double speed) { elevpower = speed; }

bool Ballevator::IntakeSensor() { return !intakesens.Get(); }

bool Ballevator::SecondSensor() { return !elevsens.Get(); }

void Ballevator::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
