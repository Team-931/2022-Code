// This will store functionalities for the turret on the bot
#include <ctre/Phoenix.h>
#include <rev/cansparkmax.h>
#pragma once

#include <frc/encoder.h>
#include <frc2/command/SubsystemBase.h>

class Turret : public frc2::SubsystemBase {
 public:
  Turret();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void RotateTurret(double speed);
  void ShooterSpeed(double speed);
  void AutoTarget(bool auto_yaw, bool auto_speed);

 private:
  rev::CANSparkMax rotator;       // motor used to rotate the turret
  rev::CANSparkMax anglechanger;  // used to modify the angle at which the
                                  // turret is at (up/down in degrees)
  rev::SparkMaxRelativeEncoder rotPos, elevPos;
  rev::SparkMaxPIDController elevCtrl;
  WPI_TalonFX shooterL, shooterR;
};