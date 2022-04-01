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

double EstimateAngleFromCamera(double ty);

Turret::Turret()
    : rotator(turretrotator, rev::CANSparkMax::MotorType::kBrushless),
      anglechanger(turretangler, rev::CANSparkMax::MotorType::kBrushless),
      rotPos(rotator.GetEncoder()),
      elevPos(anglechanger.GetEncoder()),
      elevCtrl(anglechanger.GetPIDController()),
      shooterL(shooterLeft),
      shooterR(shooterRight),
      shooterSpeed(shooterSpdInit) {
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
  anglechanger.SetInverted(true);
  elevCtrl.SetP(elevCtlP);
  elevCtrl.SetI(elevCtlI);
  shooterL.Follow(shooterR);
  shooterL.SetInverted(TalonFXInvertType::OpposeMaster);
  shooterR.SetInverted(TalonFXInvertType::Clockwise);
  shooterR.Config_kP(0, shooterCtlP);
  shooterR.Config_kI(0, shooterCtlI);
}

void Turret::Periodic() {
  static int delayer =0;
  if((delayer++) % 8 == 0) {
    frc::SmartDashboard::PutNumber("rotator position", rotPos.GetPosition());
  frc::SmartDashboard::PutNumber("shooterSpeed(target)", shooterSpeed);
  frc::SmartDashboard::PutNumber("shooterSpeed(actual)",
                                 shooterR.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("shooterAngle(target)", shooterAngle);
  }
  elevCtrl.SetReference(shooterAngle, rev::CANSparkMax::ControlType::kPosition);
}

void Turret::Fire(bool isFiring) {
  if (isFiring) {
    shooterR.Set(TalonFXControlMode::Velocity, shooterSpeed);
  } else {
    shooterR.Set(0.0);  // Power zero, skipping PID
  }
}

bool Turret::ReadyToFire() {
  return shooterR.GetSelectedSensorVelocity() > 0.98 * shooterSpeed;
}

void Turret::RotateTurret(double rate) { rotator.Set(rate * rotatorpower); }

// Adjust the "angle changer" position
void Turret::AdjustAngle(double rate) {
  shooterAngle += rate * 2.0;
  shooterAngle = std::clamp(shooterAngle, elevMin, elevMax);
}

// Set the shooter speed (from -1.0 to +1.0)
void Turret::AdjustSpeed(double rate) {
  // The motor's maximum rated speed is 6,380 RPM, the encoder has
  // 2,048 counts/revolution, and motor velocities are in units of
  // "counts per 100ms", so the theoretical max speed is 21,777.
  shooterSpeed += rate * 100.0;
}

// The lookup table contains "infinitely low" and "infinitely high"
// points so that the indexing math can work. By repeating the first
// and last "empirical" values at those points, we make the logic
// flatten out after that point.
#define DBL_MAX std::numeric_limits<double>::max()
const std::vector<double> auto_pitch_ty = {-DBL_MAX, -24.48, -18.04,
                                           -8.39,    15.8,   DBL_MAX};
const std::vector<double> auto_pitch_speed = {18542, 18542, 16305,
                                              12984, 11000, 11000};
const std::vector<double> auto_pitch_angle = {25.48, 25.48, 18.81,
                                              15.93, 10.48, 10.48};

// Find the first index for which the input 'ty' is less than
// the lookup table value.
int AutoPitchIndex(double ty) {
  for (int idx = 0; idx < (int)auto_pitch_ty.size(); idx++) {
    if (ty < auto_pitch_ty[idx]) {
      return idx;
    }
  }
  return auto_pitch_ty.size() - 1;
}

#define AUTOTARGET_SMOOTHING 4

void Turret::AutoTarget(bool auto_yaw, bool auto_pitch) {
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

  if (auto_pitch) {
    int idx = AutoPitchIndex(ty);

    double ty_a = auto_pitch_ty[idx - 1];
    double ty_b = auto_pitch_ty[idx];
    double alpha = (ty - ty_a) / (ty_b - ty_a);

    double angle_a = auto_pitch_angle[idx - 1];
    double angle_b = auto_pitch_angle[idx];
    double angle = angle_a * (1.0 - alpha) + angle_b * alpha;
    shooterAngle = (shooterAngle * (AUTOTARGET_SMOOTHING - 1) + angle) /
                   AUTOTARGET_SMOOTHING;

    double speed_a = auto_pitch_speed[idx - 1];
    double speed_b = auto_pitch_speed[idx];
    double speed = speed_a * (1.0 - alpha) + speed_b * alpha;
    shooterSpeed = .95*(shooterSpeed * (AUTOTARGET_SMOOTHING - 1) + speed) /
                   AUTOTARGET_SMOOTHING;

#if defined(AUTOTARGET_DEBUG)
    frc::SmartDashboard::PutNumber("Alpha", alpha);
    frc::SmartDashboard::PutNumber("TargetY[A]", ty_a);
    frc::SmartDashboard::PutNumber("TargetY[B]", ty_b);
    frc::SmartDashboard::PutNumber("Angle[A]", angle_a);
    frc::SmartDashboard::PutNumber("Angle[B]", angle_b);
    frc::SmartDashboard::PutNumber("Speed[A]", speed_a);
    frc::SmartDashboard::PutNumber("Speed[B]", speed_b);
    frc::SmartDashboard::PutNumber("Auto Angle", angle);
    frc::SmartDashboard::PutNumber("Auto Speed", speed);
#endif /* AUTOTARGET_DEBUG */
  }
}
