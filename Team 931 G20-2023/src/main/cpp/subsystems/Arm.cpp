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
  stage1.SetNeutralMode(Brake);
  stage2.SetNeutralMode(Brake);
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

void Arm::SetAngles(double deg1, double deg2) {
  deg1 /= 360; deg2 /= 360;
  double g2 = gravCompensator * momentStage2 * sin (2*pi*(deg2 - nAngleStage2));
  double g1 = gravCompensator * momentStage1 * sin (2*pi*(deg1 - nAngleStage1)) + g2;
  stage1.Set(ControlMode::MotionMagic, deg1 * ticksPerRotation, 
    DemandType::DemandType_ArbitraryFeedForward, g1);
  stage2.Set(ControlMode::MotionMagic, (deg2 + deg2) * ticksPerRotation, 
    DemandType::DemandType_ArbitraryFeedForward, -g2);
}
