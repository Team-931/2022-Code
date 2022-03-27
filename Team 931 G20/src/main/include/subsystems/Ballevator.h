// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/cansparkmax.h>

class Ballevator : public frc2::SubsystemBase {
 public:
  Ballevator();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetSpeed(double speed);
  bool IntakeSensor();
  bool SecondSensor();

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  frc::DigitalInput intakesens;
  frc::DigitalInput elevsens;
  rev::CANSparkMax elevator;
  float elevpower = 0;
};
