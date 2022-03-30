// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <string>

#include "Constants.h"
using namespace Constants::DriveTrain;

DriveTrain::DriveTrain() {
  // Implementation of subsystem constructor goes here.
  SetName("drive train");
}

// Values determined empirically by wiggling the joystick
// around. If you find that it tends to "stick" when released
// or jolt around unexpectedly, you may need to increase these.
const double JOYSTICK_MOTION_DEADZONE = 0.1;
const double JOYSTICK_ROTATION_DEADZONE = 0.2;

// Joysticks are noisy inputs and you need to ignore everything
// below some threshold and just clamp it to zero. This function
// zeroes out small inputs (say, +/- <dz>) and rescales the ranges
// (-1.0, <dz>] and [<dz>, 1.0) to (-1.0, 0.0) and (0.0, 1.0) so
// that every possible signal can still be input outside of the
// dead zone.
double CalculateDeadZone(double deadzone, double x) {
  if (std::abs(x) < deadzone) {
    return 0;
  }
  if (x > 0) {
    x = (x - JOYSTICK_MOTION_DEADZONE) / (1.0 - JOYSTICK_MOTION_DEADZONE);
  } else {
    x = (x + JOYSTICK_MOTION_DEADZONE) / (1.0 - JOYSTICK_MOTION_DEADZONE);
  }
  return x;
}

// Square the input except preserving the input sign, so
// that for instance -0.5 becomes -0.25 rather than 0.25.
double QuadraticScaling(double x) { return (x < 0.0 ? -1.0 : 1.0) * x * x; }

void DriveTrain::SetVforTeleop(double linX, double linY, double rot, double throttle,
                      bool fieldctr) {
  // Dead zones
  linX = CalculateDeadZone(JOYSTICK_MOTION_DEADZONE, linX);
  linY = CalculateDeadZone(JOYSTICK_MOTION_DEADZONE, linY);
  rot = CalculateDeadZone(JOYSTICK_ROTATION_DEADZONE, rot);

  // Quadratic scaling on inputs
  linX = QuadraticScaling(linX);
  linY = QuadraticScaling(linY);
  rot = QuadraticScaling(rot);

  SetV(linX, linY, rot, throttle, fieldctr);

}

void DriveTrain::SetV(double linX, double linY, double rot, double throttle,
                      bool fieldctr) {
#ifdef DRIVE_TRAIN_DEBUG
  frc::SmartDashboard::PutNumber("linX", linX);
  frc::SmartDashboard::PutNumber("linY", linY);
  frc::SmartDashboard::PutNumber("rot", rot);
#endif /* DRIVE_TRAIN_DEBUG */
  if (fieldctr) {
    // todo:
    double yaw = -wpi::numbers::pi / 180 * navx.GetYaw();
    double c = std::cos(yaw), s = std::sin(yaw);
    double x = linX * c - linY * s, y = linX * s + linY * c;
    linX = x;
    linY = y;
  }

  double scale = 1 / throttle;
  for (auto& wheel : wheels)
    scale = std::max(scale, wheel.SetV(linX, linY, rot));
  for (auto& wheel : wheels) wheel.ScaleV(scale);
}
// returns in inches the sigh is not reliable
double DriveTrain::Distance(int ix) {
  return wheels[ix].Distance()/ ticksPerInch;
}

double SwerveModule::Distance() {
  return drive.GetSelectedSensorPosition();
}

double DriveTrain::Yaw() {
  return navx.GetYaw();
}

void DriveTrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveTrain::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

int SwerveModule::ix = 0;

SwerveModule::SwerveModule()
    : drive(drvnum[ix]),
      turn(trnnum[ix]),
      absAngle(encodernum[ix]),
      index(ix),
      offsetX(offsetXs[ix]),
      offsetY(offsetYs[ix]) {
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
  drive.ConfigOpenloopRamp(0);//ask how much
  drive.ConfigClosedloopRamp(.5);//ask how much
}

double SwerveModule::SetV(double linX, double linY, double rot) {
  rot /= rotationRescale;
  linX += offsetY * rot;
  linY -= offsetX * rot;
  double spd = speed = std::sqrt(linX * linX + linY * linY),
         ang = std::atan2(linY, linX);
  // this puts angle into same semicircle as its old value,
  // equivalent to adding or subtracting multiples of 180 degrees to ang to keep
  // it as close to angle as possible. Note that if we change ang by an odd
  // number of semicircles we change the sign of speed.
  int phase;
  angle = angle - std::remquo(angle - ang, wpi::numbers::pi, &phase);
  if (phase & 1) speed = -speed;
  // double oldangle = turn.GetSelectedSensorPosition();//maybe later if we want
  // to refer to actual position
  turn.Set(ControlMode::Position, ticksPerRadian * angle);
  return spd;
}

void SwerveModule::ScaleV(double scale) {
  speed /= scale;  // we assert scale >= 1
                   // frc::SmartDashboard::PutNumber ("speed", speed);
}

void SwerveModule::Periodic() {
  // Implementation of subsystem periodic method goes here.
  drive.Set(speed);
#ifdef DRIVE_TRAIN_DEBUG
  static int ctr = 0;
  if ((ctr++) % 5 == 0) {
    double ang = absAngle.GetAbsolutePosition();
    frc::SmartDashboard::PutNumber(GetName() + " abs Encoder", 4096 * ang);
    frc::SmartDashboard::PutNumber(
        GetName() + " encoder diff",
        turn.GetSelectedSensorPosition() + ticksPerAbsTick * ang);
  }
#endif /* DRIVE_TRAIN_DEBUG */
}
void DriveTrain::Init() {
  for (auto& wheel : wheels) wheel.Init();
}

void SwerveModule::Init() {
  turn.SetSelectedSensorPosition(
      ticksPerAbsTick *
      (absSubtraction[index] / 4096. - absAngle.GetAbsolutePosition()));
}

void SwerveModule::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}