// This will store the definitions of the Turret's functionalities stored in
// Turret.h

#include "subsystems/Turret.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/span.h>

#include "Constants.h"

using namespace Constants::Turret;

Turret::Turret()
    : rotator(turretrotator, rev::CANSparkMax::MotorType::kBrushless),
      anglechanger(turretangler, rev::CANSparkMax::MotorType::kBrushless),
      rotPos(rotator.GetEncoder()),
      elevPos(anglechanger.GetEncoder()),
      elevCtrl(anglechanger.GetPIDController()),
      shooterL(shooterLeft),
      shooterR(shooterRight),
      shooterSpeed(0.0) {
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
  // shooterR.Config_kP(0, .1);
}

void Turret::Periodic() {
  frc::SmartDashboard::PutNumber("shooterSpeed", shooterSpeed);
  shooterR.Set(TalonFXControlMode::PercentOutput, shooterSpeed);
}

void Turret::RotateTurret(double speed) { rotator.Set(speed * rotatorpower); }

void Turret::ShooterSpeed(double speed) { shooterSpeed = speed; }

void Turret::AutoTarget(bool auto_yaw, bool auto_speed) {
  std::shared_ptr<nt::NetworkTable> table =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-chaos");
  double tx = table->GetNumber("tx", 0.0);
  double ty = table->GetNumber("ty", 0.0);
  double targetArea = table->GetNumber("ta", 0.0);
  double targetSkew = table->GetNumber("ts", 0.0);
  frc::SmartDashboard::PutNumber("camera_tx", tx);
  frc::SmartDashboard::PutNumber("camera_ty", ty);
  frc::SmartDashboard::PutNumber("target_area", targetArea);

  if (auto_yaw) {
    double rotate = 0.0;
    if (targetArea > 0) {
      rotate = -0.05 * tx;
    } else {
      // TODO(wgd): In principle, we could use the gyro (which must
      // already be used by the field-centric drive system) to figure
      // out where the center of the field is, and default to "face
      // the center of the field" when there's no target tracking.
      // But that's more work than just homing back to "face ahead".
      rotate = -0.008 * rotPos.GetPosition();
    }
    rotate = std::clamp(rotate, -1.0, 1.0);
    rotator.Set(rotate);
  }

  if (auto_speed) {
    // TODO(wgd): Use height/distance measurement from camera,
    // look up against a table of empirically determined values
    // (probably interpolating), and apply that speed/power to
    // the motor.
  }
}
