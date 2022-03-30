# include "commands/AutoDrive.h"
# include "subsystems/DriveTrain.h"

AutoDrive::AutoDrive(DriveTrain & d, double x, double y, double spd) : drive(d) {
    dist = std::sqrt(x*x+y*y);
    vx = x / dist * spd;
    vy = y / dist * spd;
}

void AutoDrive::Initialize() {
    startEnc = drive.Yaw();
}

bool AutoDrive::IsFinished() {
    return std::abs (drive.Yaw() - startEnc) >= dist;
}

void AutoDrive::Execute() {
    drive.SetV(vx, vy, 0, 1);
}
