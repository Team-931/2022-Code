// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
# include <frc2/command/button/JoystickButton.h>
# include <frc2/command/SequentialCommandGroup.h>
# include <frc2/command/ParallelRaceGroup.h>
# include <frc2/command/WaitCommand.h>

#include "Constants.h"
using namespace Constants::RobotContainer;

# include "commands/autoaim.h"
# include "commands/AutoDrive.h"

class rot : public frc2::CommandHelper<frc2::CommandBase, rot> {
  DriveTrain & drv;
  double speed;
  double tgt;
  double start;
  public:
  rot(DriveTrain& d, double spd, double target) : drv(d), speed(spd), tgt(target) {
    AddRequirements(&d);
  }
  void Initialize() {start = drv.Yaw();}
  bool IsFinished() {return std::remainder (std::abs(drv.Yaw() - tgt), 360) < 1;}
  void Execute() {drv.SetV(0,0,speed,1);}
};

RobotContainer::RobotContainer()
    : m_autonomousCommand(intake), drivebyStick(drivetrain, *this) {
  // Initialize all of your commands and subsystems here
  chooser.SetDefaultOption("run intake", &m_autonomousCommand);
  chooser.AddOption("rotate only", new rot(drivetrain, .25, 90));
  chooser.AddOption("drive only", new AutoDrive(drivetrain,36,0,.25));
  chooser.AddOption("rotate, shoot, drive", 
    new frc2::SequentialCommandGroup (
      rot(drivetrain, .25, 170),
      frc2::WaitCommand(1.0_s),
      autoaim(turret, ballevator).WithTimeout(5.0_s),
      AutoDrive(drivetrain,48,0,.25)
      ));
  chooser.AddOption("drive, rotate, shoot", 
    new frc2::SequentialCommandGroup (
      AutoDrive(drivetrain,48,0 ,.25),
      rot(drivetrain, .25, 170),
      frc2::WaitCommand(1.0_s),
      autoaim(turret, ballevator).WithTimeout(5.0_s)
      ));
  frc::SmartDashboard::PutData(& chooser);
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
  frc2::JoystickButton(&drivestickJ, 7).WhenPressed([this]() {drivetrain.ResetYaw();});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
    return chooser.GetSelected();
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
