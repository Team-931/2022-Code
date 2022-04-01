
#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/cansparkmax.h>

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetDeployed(bool deployed);
  bool IsDeployed();

  void SetReversed(bool);

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax wheels;
  frc::DoubleSolenoid raiser;
  bool deployed, reversed;
};
