
#pragma once

# include <frc2/command/SubsystemBase.h>
# include <frc/Solenoid.h>
# include <rev/cansparkmax.h>   

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * true starts intake; false stops it.
  */
  void startstop (bool);
  
  /**
   * true raises intake; false lowers it.
  */
  void raiselower (bool);  
 
  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax wheels;
  frc::Solenoid raiser;
  bool running;
};
