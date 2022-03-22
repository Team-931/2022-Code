// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

# include "commands/ExampleCommand.h"
# include "subsystems/Intake.h"
# include "subsystems/DriveTrain.h"
# include "subsystems/Turret.h"
# include <frc/XboxController.h> //this is the file containing connection to xbox controller

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The robot's subsystems and commands are defined here...
  Intake intake;
  DriveTrain drivetrain;
  Turret turret;
  ExampleCommand m_autonomousCommand;

  void ConfigureButtonBindings();

  struct DrvbyStick
    : public frc2::CommandHelper<frc2::CommandBase, DrvbyStick> {
        DrvbyStick(DriveTrain & d, frc::XboxController & j) : it(d), joy(j) {
          AddRequirements (&d);
        }
        void Execute() override {
          if (joy.GetYButton()) it.SetV(0,0,0);
          else
          if (joy.GetXButton()) it.SetV(.25,0,0);
          else
          if (joy.GetAButton()) it.SetV(0,.25,0);
          else
          if (joy.GetBButton()) it.SetV(-.25,0,0);
          else
          it.SetV (-joy.GetLeftY(), joy.GetLeftX(), joy.GetRightY());
        }
        DriveTrain & it;
        frc::XboxController & joy;
    }
    drivebyStick {drivetrain, driverstick};
  //The driver's controller (for manual control)
  frc::XboxController driverstick{0}; //0 is only temporary (controller responsible for moving the robot)
  
  frc::XboxController operatorstick{1}; //1 is only temporary (controller responsible for shooting the ball)
};
