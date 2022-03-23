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
# include <frc/Joystick.h>

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

  void Init();

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
  struct TurbyStick
    : public frc2::CommandHelper<frc2::CommandBase, TurbyStick>  {
        TurbyStick(Turret & t, frc::XboxController & j) : it(t), joy(j) {
          AddRequirements (&t);
        }
        void Execute() override { // constantly waiting  for input for turret

//modifying turret rotation
          if(joy.GetXButton()) it.RotateTurret(.1); // if the x button is pressed, then rotate the turret to the left(counterclockwise)
          else
          if(joy.GetBButton()) it.RotateTurret(-.1); // if the b button is pressed, then rotate the turret to the right(clockwise)
          else
           it.RotateTurret(0); //otherwise do nothing

//modifying turret angle (elevation)
          if(joy.GetYButton()) it.ModifyAngle(-0.1); // if the Y button is pressed, then angle the turret upwards
          else
          if(joy.GetAButton()) it.ModifyAngle(0.1); // if the A button is pressed, then angle the turret downwards
          else
           it.ModifyAngle(0); //otherwise do nothing
// shoot??
          it.ShootTheBall (joy.GetRightBumper());
        }
        Turret & it;
        frc::XboxController & joy;
  } turretbyStick {turret, operatorstick};
  //The driver's controller (for manual control)
  frc::XboxController driverstick{0}; //0 is only temporary (controller responsible for moving the robot)
  frc::Joystick drivestickJ{0};
  
  frc::XboxController operatorstick{1}; //1 is only temporary (controller responsible for shooting the ball)
};
