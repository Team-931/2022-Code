// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

# include <cmath>
# include <string>
# include "subsystems/DriveTrain.h"
# include "Constants.h"
# include <frc/smartdashboard/SmartDashboard.h>
using namespace Constants::DriveTrain;

DriveTrain::DriveTrain() {
  // Implementation of subsystem constructor goes here.
  SetName("drive train");
}

void DriveTrain::SetV(double linX, double linY, double rot,
  double throttle, bool fieldctr) {
    //frc::SmartDashboard::PutNumber("throttle", throttle);
    if (fieldctr) {
        // todo:
        double yaw = - wpi::numbers::pi / 180 * navx.GetYaw();
        double c = std::cos(yaw), s = std::sin(yaw);
        double x = linX*c - linY*s, y = linX*s + linY*c;
        linX = x; linY = y;
    }
    double scale = 1/throttle;
    for (auto & wheel : wheels) scale = std::max (scale, wheel.SetV(linX, linY, rot));
    for (auto & wheel : wheels) wheel.ScaleV (scale);
    }

void DriveTrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveTrain::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

int SwerveModule::ix = 0;

SwerveModule::SwerveModule() : drive (drvnum[ix]), turn (trnnum[ix]),
 absAngle (encodernum[ix]), index (ix),
 offsetX (offsetXs[ix]), offsetY (offsetYs[ix]) {
    SetName("wheels " + std::to_string(ix));
    AddChild("absAngle", &absAngle);
    ++ix;
    turn.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    turn.SetNeutralMode(NeutralMode::Coast);
    // does this work?
/*     turn.ConfigIntegratedSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    turn.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Signed_PlusMinus180);
 */    // set PID
    turn.Config_kP(0, .25);
    drive.SetNeutralMode(NeutralMode::Brake);
}

double SwerveModule::SetV(double linX, double linY, double rot) {
    rot /= rotationRescale;
    linX += offsetY * rot;
    linY -= offsetX * rot;
    double spd = speed = std::sqrt (linX*linX + linY*linY),
      ang = std::atan2 (linY, linX);
    // this puts angle into same semicircle as its old value,
    // equivalent to adding or subtracting multiples of 180 degrees to ang to keep it as close to angle as
    // possible. Note that if we change ang by an odd number of semicircles we change the sign of speed.
    int phase;
    angle = angle - std::remquo (angle - ang, wpi::numbers::pi, &phase);
    if (phase & 1) speed = - speed;
    //double oldangle = turn.GetSelectedSensorPosition();//maybe later if we want to refer to actual position
    turn.Set (ControlMode::Position, ticksPerRadian * angle);
    return spd;
}

void SwerveModule::ScaleV(double scale) {
    speed /= scale; // we assert scale >= 1
    //frc::SmartDashboard::PutNumber ("speed", speed); 
}

void SwerveModule::Periodic() {
  // Implementation of subsystem periodic method goes here.
  drive.Set(speed);
  static int ctr = 0;
  if ((ctr++) % 5 == 0) {
    double ang = absAngle.GetAbsolutePosition();
    frc::SmartDashboard::PutNumber(GetName() + " abs Encoder", 4096*ang);
    frc::SmartDashboard::PutNumber(GetName() + " encoder diff", turn.GetSelectedSensorPosition() + ticksPerAbsTick * ang);
  }
}
void DriveTrain::Init() {
  for (auto & wheel : wheels) wheel.Init();
}

void SwerveModule::Init() {
  turn.SetSelectedSensorPosition(ticksPerAbsTick * (absSubtraction[index]/4096. - absAngle.GetAbsolutePosition()));
}

void SwerveModule::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}