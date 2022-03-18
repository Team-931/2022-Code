// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
# include "Constants.h"
using namespace Constants::Intake;

Intake::Intake() : wheels (whnum, rev::CANSparkMax::MotorType::kBrushless),
 actuator(frc::PneumaticsModuleType::CTREPCM, actnum),
 running (false) {
  // Implementation of subsystem constructor goes here.
}

void Intake::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void Intake::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
