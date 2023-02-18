// This will store functionalities for the turret on the bot
#include <ctre/Phoenix.h>
#pragma once

#include <frc/encoder.h>
#include <frc2/command/SubsystemBase.h>

class Arm : public frc2::SubsystemBase {
 public:
  Arm();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetAngles(double stage1Degrees, double stage2Degrees); // 0, 0 is start position 
 private:
  WPI_TalonFX stage1, stage2;
};