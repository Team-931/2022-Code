// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

# include "subsystems/DriveTrain.h"
# include "Constants.h"
using namespace Constants::DriveTrain;

DriveTrain::DriveTrain() {
  // Implementation of subsystem constructor goes here.
}

void DriveTrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveTrain::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

SwerveModule::SwerveModule(int ix) : drive (drvnum[ix]), turn (trnnum[ix]) {}