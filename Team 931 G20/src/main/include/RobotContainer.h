// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>  //this is the file containing connection to xbox controller
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>

#include "commands/ExampleCommand.h"
#include "subsystems/Ballevator.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Intake.h"
#include "subsystems/Turret.h"

#define BALLEVATOR_IDLE 0
#define BALLEVATOR_READY 1
#define BALLEVATOR_LOADING 2
#define BALLEVATOR_HOLD 3
#define BALLEVATOR_FIRE 4

const double BALLEVATOR_SPEED_IDLE = 0.0;
const double BALLEVATOR_SPEED_READY = 0.75;
const double BALLEVATOR_SPEED_LOADING = 0.65;
const double BALLEVATOR_SPEED_HOLD = 0.0;
const double BALLEVATOR_SPEED_FIRE = 1.0;

const double TURRET_YAW_DEADZONE = 0.25;
const double TURRET_ANGLE_DEADZONE = 0.2;
const double TURRET_SPEED_DEADZONE = 0.2;

/* HACK(wgd): This should probably live somewhere more 'utilities-ish'
   and not just be forward-declared here, but this is faster. */
double CalculateDeadZone(double deadzone, double x);

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
  bool XBox{false};
  ExampleCommand m_autonomousCommand;

  void ConfigureButtonBindings();

  struct DrvbyStick
      : public frc2::CommandHelper<frc2::CommandBase, DrvbyStick> {
    DrvbyStick(DriveTrain& d, RobotContainer& j) : it(d), bot(j) {
      AddRequirements(&d);
    }
    void Execute() override {
      static bool fieldcentered = true;
      if (bot.GetFieldCenterToggle()) fieldcentered ^= true;
      // todo: add throttle
      it.SetV(bot.GetX(), bot.GetY(), bot.GetRot(), bot.GetThrottle(),
              fieldcentered);
    }
    DriveTrain& it;
    RobotContainer& bot;
  } drivebyStick;

  struct TurbyStick
      : public frc2::CommandHelper<frc2::CommandBase, TurbyStick> {
    TurbyStick(Turret& t, frc::XboxController& j) : it(t), joy(j) {
      AddRequirements(&t);
    }
    void Execute() override {
      // State Variables
      static bool auto_target = true;
      frc::SmartDashboard::PutBoolean("auto_target", auto_target);

      // Operator inputs
      bool firefirefire = joy.GetAButton();
      bool auto_target_toggle = joy.GetBButtonPressed();
      double turret_manual_yaw =
          CalculateDeadZone(TURRET_YAW_DEADZONE, -joy.GetRightX());
      double turret_manual_angle =
          CalculateDeadZone(TURRET_ANGLE_DEADZONE, -joy.GetRightY());
      double turret_manual_speed =
          CalculateDeadZone(TURRET_SPEED_DEADZONE, -joy.GetLeftY());
      frc::SmartDashboard::PutNumber("manual_yaw", turret_manual_yaw);
      frc::SmartDashboard::PutNumber("manual_angle", turret_manual_angle);
      frc::SmartDashboard::PutNumber("manual_speed", turret_manual_speed);

      // State update
      if (auto_target_toggle) {
        auto_target = !auto_target;
      }

      // Control robot
      it.Fire(firefirefire);

      bool auto_target_yaw = false;
      if (turret_manual_yaw != 0.0) {
        it.RotateTurret(turret_manual_yaw);
      } else if (auto_target) {
        auto_target_yaw = true;
      }

      it.AdjustSpeed(turret_manual_speed);

      bool auto_target_pitch = false;
      if (turret_manual_angle != 0.0) {
        it.AdjustAngle(turret_manual_angle);
      } else if (auto_target) {
        auto_target_pitch = true;
      }

      it.AutoTarget(auto_target_yaw, auto_target_pitch);
    }
    Turret& it;
    frc::XboxController& joy;
  } turretbyStick{turret, operatorstick};

  struct IntbyStick
      : public frc2::CommandHelper<frc2::CommandBase, IntbyStick> {
    IntbyStick(Intake& i, frc::XboxController& j) : it(i), joy(j) {
      AddRequirements(&i);
    }
    void Execute() override {
      // State Variables
      static bool intake_deployed = false;
      frc::SmartDashboard::PutBoolean("intake_deployed", intake_deployed);

      // Operator Inputs
      bool intake_deploy = joy.GetRightBumper();
      bool intake_raise = joy.GetLeftBumper();

      // State update logic
      if (intake_deploy && !intake_raise) {
        intake_deployed = true;
      }
      if (intake_raise && !intake_deploy) {
        intake_deployed = false;
      }

      // Control robot
      it.SetDeployed(intake_deployed);
    }
    Intake& it;
    frc::XboxController& joy;
  } Intakebystick{intake, operatorstick};

  struct Ballevate : public frc2::CommandHelper<frc2::CommandBase, Ballevate> {
    Ballevate(Ballevator& b, frc::XboxController& j) : it(b), joy(j) {
      AddRequirements(&b);
    }
    void Execute() override {
      // State Variables
      static int ballevator_state = BALLEVATOR_IDLE;
      frc::SmartDashboard::PutString(
          "ballevator_state",
          ballevator_state == BALLEVATOR_IDLE      ? "IDLE"
          : ballevator_state == BALLEVATOR_READY   ? "READY"
          : ballevator_state == BALLEVATOR_LOADING ? "LOADING"
          : ballevator_state == BALLEVATOR_HOLD    ? "HOLD"
          : ballevator_state == BALLEVATOR_FIRE    ? "FIRE"
                                                   : "UNKNOWN");

      // Operator and Sensor Inputs
      bool intake_sensor = it.IntakeSensor();
      bool second_sensor = it.SecondSensor();
      bool firefirefire = joy.GetAButton();
      frc::SmartDashboard::PutBoolean("intake_sensor", intake_sensor);
      frc::SmartDashboard::PutBoolean("second_sensor", second_sensor);

      // This is a bit ugly, but I'm not familiar enough with this whole
      // "subsystems and commands" framework to do it better. All we need
      // here is to take into account whether the intake is deployed so
      // that the elevator can turn off when it's not needed, but the
      // easiest way I could find to do that was to copy-paste the same
      // state update logic instead of just *asking* the relevant subsystem.
      static bool intake_deployed = false;
      bool intake_deploy = joy.GetRightBumper();
      bool intake_raise = joy.GetLeftBumper();
      if (intake_deploy && !intake_raise) {
        intake_deployed = true;
      }
      if (intake_raise && !intake_deploy) {
        intake_deployed = false;
      }

      // State update + robot control logic
      if (firefirefire) {
        ballevator_state = BALLEVATOR_FIRE;
        it.SetSpeed(BALLEVATOR_SPEED_FIRE);
      } else if (second_sensor) {
        ballevator_state = BALLEVATOR_HOLD;
        it.SetSpeed(BALLEVATOR_SPEED_HOLD);
      } else if (intake_sensor) {
        ballevator_state = BALLEVATOR_LOADING;
        it.SetSpeed(BALLEVATOR_SPEED_LOADING);
      } else if (ballevator_state == BALLEVATOR_IDLE && intake_deployed) {
        ballevator_state = BALLEVATOR_READY;
        it.SetSpeed(BALLEVATOR_SPEED_READY);
      } else if (ballevator_state == BALLEVATOR_READY && !intake_deployed) {
        ballevator_state = BALLEVATOR_IDLE;
        it.SetSpeed(BALLEVATOR_SPEED_IDLE);
      } else if (ballevator_state == BALLEVATOR_FIRE && !firefirefire) {
        ballevator_state = BALLEVATOR_READY;
        it.SetSpeed(BALLEVATOR_SPEED_READY);
      }
    }
    Ballevator& it;
    frc::XboxController& joy;
  } Ballelevate{ballevator, operatorstick};

  // The driver's controller (for manual control)
  frc::XboxController driverstick{
      0};  // 0 is only temporary (controller responsible for moving the robot)
  frc::Joystick drivestickJ{0};

  frc::XboxController operatorstick{
      1};  // 1 is only temporary (controller responsible for shooting the ball)
};
