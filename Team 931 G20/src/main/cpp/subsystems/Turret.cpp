//This will store the definitions of the Turret's functionalities stored in Turret.h


#include "subsystems/Turret.h"
#include "Constants.h"

using namespace Constants::Turret;
        
Turret::Turret() : rotator(turretrotator, rev::CANSparkMax::MotorType::kBrushless), 
anglechanger(turretangler, rev::CANSparkMax::MotorType::kBrushless) {
// Implementation of Turret constructor.
rotator.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
anglechanger.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Turret::Periodic() {
// Implementation of Turret periodic method.


}


void Turret::ShootTheBall(){
//Shoots the ball into the basket

} 


void Turret::ModifyAngle(double power ){ //for now, this function works based on the power sent to it
//Modifies the angle at which the turret is set (up down/ how much it is it angled)

anglechanger.Set(power);



} 


void Turret::RotateTurret( double power){//for now, this function works based on the power sent to it
//rotates the turret (has a set amount of degrees at which it can be turned based on motor power)
  
    rotator.Set(power); 

} 

