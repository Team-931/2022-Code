// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

# include <frc2/command/SubsystemBase.h>
# include <ctre/Phoenix.h>
# include <AHRS.h>

/**
 * All the control objects for a single wheel. Initialized with an index to Id. tables in constants.h.
*/
class SwerveModule : public frc2::SubsystemBase {
  public:
    SwerveModule(int);
/**
 * the module's motion is set relative to the robot's axes
*/

  void SetVelocities (double linX, double linY, double rot); 

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
  double offsetX, offsetY;
};

/**
 * all the driving and navigation objects, incl. a SwerveModule for each wheel.
*/
class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
//todo: PID?? to set orientation during auto

  /**
   * Set linear and rotational velocity relative to the field by default, otherwise relative to the robot.                                          
   */
  void SetV (double linX, double linY, double rot,  bool fieldcentered = true);
  
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
  AHRS navx {frc::SPI::Port::kMXP};
};
