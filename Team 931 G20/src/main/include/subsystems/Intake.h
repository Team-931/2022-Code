
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
  void startstop (bool);
  void raiselower (bool );  
  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax wheels;
  frc::Solenoid actuator;
  bool running;
};
