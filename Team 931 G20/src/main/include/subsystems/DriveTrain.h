// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

# include <frc2/command/SubsystemBase.h>
# include <ctre/Phoenix.h>

/**
 * All the control objects for a single wheel. Initialized with an index to Id. tables in constants.h.
*/
class SwerveModule : public frc2::SubsystemBase {
  public:
    SwerveModule(int);
/**
 * the module's motion is set relative to the robot's axes
*/

  void SetVelocities (double linX, double linY); 

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::WPI_TalonFX drive, turn;
};

/**
 * all the driving and navigation objects, incl. a SwerveModule for each wheel.
*/
class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveModule leftFront {0}, rightFront {1}, leftRear {2}, rightRear{3};
};
