// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/cansparkmax.h>
#include <frc/AnalogInput.h>


class Ballevator : public frc2::SubsystemBase {
 public:
  Ballevator();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
  * start, stop, and rev
  * start runs towards the shooter
  * stop
  * rev runs towards the intake
  * Use the data from the distance sensors to determine true or false 
  */ 
  void startstop ();
  
  /**
    * Sensor wrangling
    * Ballevator and BV will be used interchangebly
    * 
    * Determines the location of the ball based on the voltage output of sensors 1,2, or 3
    * 
    * if sensor 1 is triggered, but not sensor 2, or sensor 3 the function should return start
    * 
    * if sensor 2 is triggered, but not sensors 1 or 3, return stop
    * 
    * if both sensors 1 and 2 are triggered, but not sensor 3 the function should return start
    * 
    * if sensors 2 and 3 are triggered, the funciton should return stop, and notify the drivers
         * that the BV is full, it should also disallow the driver from continuing to run the intake
    * 
    * if sensors 2 and 3 are triggered and the button to trigger the shooter is true, return true
    * 
    * If sensors 1, 2, and 3 are triggered, spin the intake in reverse, and run the BV in reverse
    * until 2 or fewer sensors are triggered
    *   
    */
    int ballesense();

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  frc::AnalogInput ultrasonic{0};
  rev::CANSparkMax elevator;



  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

};
