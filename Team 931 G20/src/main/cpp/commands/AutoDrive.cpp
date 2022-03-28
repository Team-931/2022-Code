# include "commands/AutoDrive.h"

AutoDrive::AutoDrive(DriveTrain & it, double x, double y) : drive(it) {
    AddRequirements(& it);
}

void AutoDrive::Initialize() {
    SwerveModule::controlMode = TalonFXControlMode::Velocity;
}

void AutoDrive::Execute() {

}