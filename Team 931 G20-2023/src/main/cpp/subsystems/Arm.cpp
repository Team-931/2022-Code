// This will store the definitions of the Arm's functionalities stored in
// Arm.h

#include "subsystems/Arm.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableValue.h>
//#include <wpi/span.h>

#include "Constants.h"

using namespace Constants::Arm;

double EstimateAngleFromCamera(double ty);

Arm::Arm()
    : stage1(stage1Id),
      stage2(stage2Id) {
  //stage1.Follow(stage2);
  stage1.SetInverted(TalonFXInvertType::Clockwise);
  stage2.SetInverted(TalonFXInvertType::Clockwise);
  stage1.Config_kP(0, CtlP);
  stage1.Config_kF(0, CtlF);
  stage1.ConfigMotionCruiseVelocity(maxVel);
  stage1.ConfigMotionAcceleration(maxAccel);
  stage2.Config_kP(0, CtlP);
  stage2.Config_kF(0, CtlF);
  stage2.ConfigMotionCruiseVelocity(maxVel);
  stage2.ConfigMotionAcceleration(maxAccel);
}

void Arm::Periodic() {
  frc::SmartDashboard::PutNumber("shooterSpeed(actual)",
                                 stage2.GetSelectedSensorVelocity());
  
}
