// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

# include "commands/ExampleCommand.h"
# include "subsystems/Intake.h"
# include "subsystems/DriveTrain.h"
# include "subsystems/Turret.h"
# include "subsystems/Ballevator.h"
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

// get forward/back from the selected joystick in our coords
  double GetX();
// get right/left from the selected joystick in our coords
  double GetY();
// get rotation from the selected joystick in our coords
  double GetRot();
// get throttle from the selected joystick in our coords
  double GetThrottle();
// 
  bool GetFieldCenterToggle();
 private:
  // The robot's subsystems and commands are defined here...
  Intake intake;
  DriveTrain drivetrain;
  Turret turret;
  Ballevator ballevator;
  bool XBox {false};
  ExampleCommand m_autonomousCommand;

  void ConfigureButtonBindings();

  struct DrvbyStick
    : public frc2::CommandHelper<frc2::CommandBase, DrvbyStick> {
        DrvbyStick(DriveTrain & d, RobotContainer & j) : it(d), bot(j) {
          AddRequirements (&d);
        }
        void Execute() override {
          static bool fieldcentered = true;
          if (bot.GetFieldCenterToggle()) fieldcentered ^= true;
          // todo: add throttle
          it.SetV (bot.GetX(), bot.GetY(), bot.GetRot(), bot.GetThrottle(), fieldcentered);
        }
        DriveTrain & it;
        RobotContainer & bot;
    }
    drivebyStick;

 struct TurbyStick
    : public frc2::CommandHelper<frc2::CommandBase, TurbyStick>  {
        TurbyStick(Turret & t, frc::XboxController & j) : it(t), joy(j) {
          AddRequirements (&t);
        }
        void Execute() override { // constantly waiting  for input for turret

//modifying turret rotation
          if(joy.GetXButton()) it.RotateTurret(1); // if the x button is pressed, then rotate the turret to the left(counterclockwise) (based on positive coefficient)
          else
          if(joy.GetBButton()) it.RotateTurret(-1); // if the b button is pressed, then rotate the turret to the right(clockwise) (based on negative coefficient)
          else
           it.RotateTurret(0); //otherwise do nothing

//modifying turret angle (elevation)
          if(joy.GetYButton()) it.ModifyAngle(-1); // if the Y button is pressed, then angle the turret upwards (based on negative coefficient)
          else
          if(joy.GetAButton()) it.ModifyAngle(1); // if the A button is pressed, then angle the turret downwards (based on positive coefficient)
          else
           it.StayAtAngle();// attempting to keep the cowl aligned at that angle 
           /*ModifyAngle(0)*/ //otherwise do nothing
// shoot??
          it.ShootTheBall (joy.GetRightBumper());

          double shootChg = joy.GetLeftY();
          if (std::abs(shootChg) >= .1) it.IncShooterSpeed(-shootChg);
        }
        Turret & it;
        frc::XboxController & joy;
  } turretbyStick {turret, operatorstick};

  struct IntbyStick
    : public frc2::CommandHelper<frc2::CommandBase, IntbyStick>  {
        IntbyStick(Intake & i, frc::XboxController & j) : it(i), joy(j) {
          AddRequirements (&i);
        }
        void Execute() override {
# if 0
          if(joy.GetLeftBumper()) {
           it.raiselower(false);
           it.startstop(true);
           } 
# else
          if(joy.GetLeftBumperPressed())
            it.toggleraiser(); 
# endif
           // if the right bumper is pressed, then lower the intake and run the wheels 
          else{
           it.startstop(false);
           it.raiselower(true);
           }
           
        }
        Intake & it;
        frc::XboxController & joy;
    } Intakebystick {intake, operatorstick};

  struct Ballevate
    : public frc2::CommandHelper<frc2::CommandBase, Ballevate>  {
        Ballevate(Ballevator & b, frc::XboxController & j) : it(b), joy(j){
          AddRequirements (&b);
        }
        void Execute() override {
//Previous  Obi code
          if(joy.GetLeftBumper())
            it.startstop(1);
          else
            it.startstop(0);


            //Nevin Update 1
            //Nevin's attempt at coding the ballevator to work based on button input
            /*
            //what sensor variables do we need to change if the ball is shot?

          if(joy.GetXButton()) // if the x button is pressed
           if(it.ballesense() <= 1) //check if there is only one ball or less in the ball ammo holder
            it.startstop(0.1); // For now set default to 0.1, will eventually be based on input though
          else
          it.set(0) // do nothing 
         

            */




            // if(joy.GetLeftBumper())
                     
                    
                     
             } // End of excecute() (ballevator)
          // else
          // if(joy.GetStartButton()) {
          //   it.startstop(3);
          // }
  
      Ballevator & it;
        frc::XboxController & joy;
    } Ballelevate {ballevator, operatorstick};
  
  //The driver's controller (for manual control)
  frc::XboxController driverstick{0}; //0 is only temporary (controller responsible for moving the robot)
  frc::Joystick drivestickJ{0};
  
  frc::XboxController operatorstick{1}; //1 is only temporary (controller responsible for shooting the ball)
};
