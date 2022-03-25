// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
# include <frc/DriverStation.h>
# include "Constants.h"
using namespace Constants::RobotContainer;

RobotContainer::RobotContainer() : m_autonomousCommand(&intake),
  drivebyStick(drivetrain, *this) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  drivetrain.SetDefaultCommand (drivebyStick);
  turret.SetDefaultCommand (turretbyStick);
  intake.SetDefaultCommand (Intakebystick);
}

void RobotContainer::Init() {
  drivetrain.Init();
  if (frc::DriverStation::GetJoystickIsXbox(0))
    XBox = true;
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}

double RobotContainer::GetX() {
  if(XBox) return driverstick.GetLeftY();
  return drivestickJ.GetY();
}

double RobotContainer::GetY() {
  if(XBox) return -driverstick.GetLeftX();
  return -drivestickJ.GetX();
}

double RobotContainer::GetRot() {
  if(XBox) return driverstick.GetRightX();
  return drivestickJ.GetTwist();
}

double RobotContainer::GetThrottle() {
  if(XBox) return (minThrottle + driverstick.GetRightTriggerAxis() * (1 - minThrottle));
  return (1 + minThrottle - drivestickJ.GetThrottle() * (1 - minThrottle)) / 2;
}

bool RobotContainer::GetFieldCenterToggle() {
  if(XBox) return driverstick.GetRightBumperPressed();
  return drivestickJ.GetTrigger();
}
