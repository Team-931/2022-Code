//This will store the definitions of the Turret's functionalities stored in Turret.h

# include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Turret.h"
#include "Constants.h"

using namespace Constants::Turret;
        
Turret::Turret() : rotator(turretrotator, rev::CANSparkMax::MotorType::kBrushless), 
    anglechanger(turretangler, rev::CANSparkMax::MotorType::kBrushless),
    rotPos(rotator.GetEncoder()), elevPos(anglechanger.GetEncoder()),
    elevCtrl(anglechanger.GetPIDController()),
    shooterL(shooterLeft), shooterR(shooterRight),
    shooterSpd(shooterSpdInit) {
// Implementation of Turret constructor.
rotator.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
rotator.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
rotator.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, rotMax);
rotator.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
rotator.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, rotMin);
anglechanger.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
anglechanger.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
anglechanger.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, elevMax);
anglechanger.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
anglechanger.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, elevMin);
elevCtrl.SetP(.1);
shooterL.Follow(shooterR);
shooterL.SetInverted(TalonFXInvertType::OpposeMaster);
shooterR.SetInverted(TalonFXInvertType::Clockwise);
shooterR.Config_kP(0,.1);
}

void Turret::Periodic() {
// Implementation of Turret periodic method.
// Testing: report positions of rotator and anglechanger
    static int counter = 0;
    if ((counter++) % 16 == 0) {
        frc::SmartDashboard::PutNumber ("rotator position", rotPos.GetPosition());
        frc::SmartDashboard::PutNumber ("anglechanger position", elevPos.GetPosition());
    }

}


void Turret::ShootTheBall(bool on){
//Shoots the ball into the basket
    shooterR.Set(TalonFXControlMode::Velocity, on ? shooterSpd : 0);
} 

void Turret::IncShooterSpeed(double inc) {
    shooterSpd += inc*100;
    frc::SmartDashboard::PutNumber("shooter speed", shooterSpd);
}

void Turret::ModifyAngle(double power ){ //for now, this function works based on the power sent to it
//Modifies the angle at which the turret is set (up down/ how much it is it angled)

anglechanger.Set(power);


} 

void Turret::StayAtAngle() {
    elevCtrl.SetReference(elevPos.GetPosition(), rev::ControlType::kPosition);
}

void Turret::RotateTurret( double power){//for now, this function works based on the power sent to it
//rotates the turret (has a set amount of degrees at which it can be turned based on motor power)
  
    rotator.Set(power); 

} 

