//This will store functionalities for the turret on the bot
# include <rev/cansparkmax.h>
# include <ctre/Phoenix.h>
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/encoder.h>

class Turret : public frc2::SubsystemBase {
 public:
  Turret();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */

  void ShootTheBall(bool = true); //Shoots the ball into the basket

  void ModifyAngle(double power); //Modifies the angle at which the turret is set (up down/ how much it is it angled)

  void RotateTurret(double power); //rotates the turret (has a set amount of degrees at which it can be turned)

     
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

   rev::CANSparkMax rotator; // motor used to rotate the turret
   rev::CANSparkMax anglechanger; //used to modify the angle at which the turret is at (up/down in degrees)
   WPI_TalonFX shooterL, shooterR;
};