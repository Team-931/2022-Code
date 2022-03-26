// This will store the definitions of the Turret's functionalities stored in
// Turret.h

#include "subsystems/Turret.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace Constants::Turret;

Turret::Turret()
    : rotator(turretrotator, rev::CANSparkMax::MotorType::kBrushless),
      anglechanger(turretangler, rev::CANSparkMax::MotorType::kBrushless),
      rotPos(rotator.GetEncoder()),
      elevPos(anglechanger.GetEncoder()),
      elevCtrl(anglechanger.GetPIDController()),
      shooterL(shooterLeft),
      shooterR(shooterRight) {
  rotator.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  rotator.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
  rotator.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, rotMax);
  rotator.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
  rotator.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, rotMin);
  anglechanger.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  anglechanger.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward,
                               true);
  anglechanger.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward,
                            elevMax);
  anglechanger.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,
                               true);
  anglechanger.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,
                            elevMin);
  elevCtrl.SetP(.1);
  shooterL.Follow(shooterR);
  shooterL.SetInverted(TalonFXInvertType::OpposeMaster);
  shooterR.SetInverted(TalonFXInvertType::Clockwise);
  shooterR.Config_kP(0, .1);
}

void Turret::Periodic() {}

void Turret::RotateTurret(double speed) { rotator.Set(speed); }

void Turret::ShooterSpeed(double speed) {
  shooterR.Set(TalonFXControlMode::Velocity, speed);
}

void Turret::AutoTarget(bool auto_yaw, bool auto_pitch) {
  // TODO: Implement this using camera
}
