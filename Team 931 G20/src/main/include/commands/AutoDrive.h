
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveTrain;

class AutoDrive
    : public frc2::CommandHelper<frc2::CommandBase, AutoDrive> {
 public:
  /**
   * Creates a new AutoDrive.
   *
   * @param subsystem The subsystem used by this command.
   * @param inchesX How many inches to move forward.
   * @param inchesY How many inches to move rightward.
   * @param speed How fast to move, currently as proportion of maximum.
   */
  AutoDrive(DriveTrain& subsystem, double inchesX, double inchesY, double speed);

  void Initialize() override;

  bool IsFinished () override;

  void Execute() override;

 private:
  DriveTrain& drive;
  double vx, vy, dist,
         startEnc;
};
