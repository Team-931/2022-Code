// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

# include <cmath>
# include "subsystems/DriveTrain.h"
# include "Constants.h"
using namespace Constants::DriveTrain;

DriveTrain::DriveTrain() {
  // Implementation of subsystem constructor goes here.
}

void DriveTrain::SetV(double linX, double linY, double rot, bool fieldctr) {
    if (fieldctr) {
        // todo:
    }
    double scale = 1;
    for (auto & wheel : wheels) scale = std::max (scale, wheel.SetV(linX, linY, rot));
    for (auto & wheel : wheels) wheel.ScaleV (scale);
}

void DriveTrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveTrain::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

SwerveModule::SwerveModule(int ix) : drive (drvnum[ix]), turn (trnnum[ix]),
 offsetX (offsetXs[ix]), offsetY (offsetYs[ix]) {
    turn.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);
}

double SwerveModule::SetV(double linX, double linY, double rot) {
    linX += offsetY * rot;
    linY -= offsetX * rot;
    speed = std::sqrt (linX*linX + linY*linY);
    angle = std::atan2 (linY, linX);
    return speed;
}

void SwerveModule::ScaleV(double scale) {
    speed /= scale; // we assert scale >= 1
}

void SwerveModule::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void SwerveModule::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}