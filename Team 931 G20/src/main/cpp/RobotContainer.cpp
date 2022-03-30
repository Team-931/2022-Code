// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
# include <frc2/command/SequentialCommandGroup.h>
# include <frc2/command/WaitCommand.h>

#include "Constants.h"
using namespace Constants::RobotContainer;

# include "commands/autoaim.h"

RobotContainer::RobotContainer()
    : m_autonomousCommand(&intake), drivebyStick(drivetrain, *this) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  drivetrain.SetDefaultCommand(drivebyStick);
  turret.SetDefaultCommand(turretbyStick);
  intake.SetDefaultCommand(Intakebystick);
  ballevator.SetDefaultCommand(Ballelevate);
}

void RobotContainer::Init() {
  drivetrain.Init();
  if (frc::DriverStation::GetJoystickIsXbox(0)) XBox = true;
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

class rot : public frc2::CommandHelper<frc2::CommandBase, rot> {
  DriveTrain & drv;
  double speed;
  double tgt;
  double start;
  public:
  rot(DriveTrain& d, double spd, double target) : drv(d), speed(spd), tgt(target) {}
  void Initialize() {start = drv.Yaw();}
  bool IsFinished() {return std::remainder (std::abs(drv.Yaw() - tgt), 360) < 1;}
  void Execute() {drv.SetV(0,0,speed,1);}
};

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  if(1) {
    return new frc2::SequentialCommandGroup (
      rot(drivetrain, .25, 170),
      frc2::WaitCommand(1.0_s),
      autoaim(turret, ballevator)
      );
    }
  return &m_autonomousCommand;
}

double RobotContainer::GetX() {
  if (XBox) return driverstick.GetLeftY();
  return drivestickJ.GetY();
}

double RobotContainer::GetY() {
  if (XBox) return -driverstick.GetLeftX();
  return -drivestickJ.GetX();
}

double RobotContainer::GetRot() {
  if (XBox) return driverstick.GetRightX();
  return drivestickJ.GetTwist();
}

double RobotContainer::GetThrottle() {
  if (XBox)
    return (minThrottle +
            driverstick.GetRightTriggerAxis() * (1 - minThrottle));
  return (1 + minThrottle - drivestickJ.GetThrottle() * (1 - minThrottle)) / 2;
}

bool RobotContainer::GetFieldCenterToggle() {
  if (XBox) return driverstick.GetRightBumperPressed();
  return drivestickJ.GetTrigger();
}
