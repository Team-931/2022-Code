// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

# include <cmath>
# include "subsystems/DriveTrain.h"
# include "Constants.h"
# include <frc/smartdashboard/SmartDashboard.h>
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
    frc::SmartDashboard::PutNumber ("linX", linX);
    frc::SmartDashboard::PutNumber ("linY", linY);
    frc::SmartDashboard::PutNumber ("ror", rot);
    frc::SmartDashboard::PutNumber ("scale", scale);
}

void DriveTrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveTrain::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

int SwerveModule::ix = 0;

SwerveModule::SwerveModule() : drive (drvnum[ix]), turn (trnnum[ix]),
 offsetX (offsetXs[ix]), offsetY (offsetYs[ix]) {
    ++ix;
    turn.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    turn.SetNeutralMode(NeutralMode::Coast);
    //todo: set PID
    turn.ConfigIntegratedSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    turn.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Signed_PlusMinus180);
    turn.Config_kP(0, .25);
    drive.SetNeutralMode(NeutralMode::Brake);
}

double SwerveModule::SetV(double linX, double linY, double rot) {
    rot /= rotationRescale;
    linX += offsetY * rot;
    linY -= offsetX * rot;
    speed = std::sqrt (linX*linX + linY*linY);
    angle = std::atan2 (linY, linX);
    turn.Set (ControlMode::Position, ticksPerRadian * angle);
    return speed;
}

void SwerveModule::ScaleV(double scale) {
    speed /= scale; // we assert scale >= 1
    //frc::SmartDashboard::PutNumber ("speed", speed); 
}

void SwerveModule::Periodic() {
  // Implementation of subsystem periodic method goes here.
  drive.Set(speed);
}

void SwerveModule::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}