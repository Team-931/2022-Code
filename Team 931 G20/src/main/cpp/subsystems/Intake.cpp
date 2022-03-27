// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include "Constants.h"
using namespace Constants::Intake;

Intake::Intake()
    : wheels(whnum, rev::CANSparkMax::MotorType::kBrushless),
      raiser(frc::PneumaticsModuleType::CTREPCM, raisenum, lownum),
      running(false) {
  // Implementation of subsystem constructor goes here.
  wheels.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Intake::Periodic() {
  // Implementation of subsystem periodic method goes here.
  if (running)
    wheels.Set(whpow);
  else
    wheels.Set(0);
}

void Intake::startstop(bool on) { running = on; }

void Intake::raiselower(bool on) {
  raiser.Set(on ? frc::DoubleSolenoid::kForward
                : frc::DoubleSolenoid::kReverse);
}

void Intake::toggleraiser() {
  raiser.Set(frc::DoubleSolenoid::kReverse == raiser.Get()
                 ? frc::DoubleSolenoid::kForward
                 : frc::DoubleSolenoid::kReverse);
}

void Intake::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
