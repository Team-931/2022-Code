// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <WPI/numbers>
#include <cmath>
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace Constants {
namespace RobotContainer {
constexpr double minThrottle = .1;
}
namespace DriveTrain {
constexpr int drvnum[]{0, 3, 4, 7}, trnnum[]{1, 2, 5, 6},
    encodernum[]{0, 1, 3, 2};
constexpr double halfLen = 29.25 / 2, halfWid = 19.75 / 2;  // X is forward
constexpr double offsetXs[]{halfLen, halfLen, -halfLen,
                            -halfLen};  // coords in inches
constexpr double offsetYs[]{halfWid, -halfWid, -halfWid,
                            halfWid};  // right front is +, +
const double rotationRescale = std::sqrt(
    halfLen * halfLen +
    halfWid * halfWid);  // todo: de-kludge this, it makes the linear and
                         // rotational control argumenrs comparable.
constexpr double turnGearing = 72.0 / 14 * 24 / 12,  // maybe use std::ratio
    ticksPerRadian = 2048 / 2 / wpi::numbers::pi * turnGearing,
                 ticksPerAbsTick =
                     turnGearing *
                     2048 /*/ 4096*/;  // todo: check this with hardware
constexpr int absSubtraction[]{3265, 2068, 135, 2673};  // to align the wheels
}  // namespace DriveTrain

namespace Intake {
constexpr int whnum{1},         /* motor, they help suck the ball in */
    raisenum{4}, lownum{0};     /* mechanism that initiates intake mechanism*/
constexpr double whpow = 0.75;  // default power for the motor whnum

}  // namespace Intake

// namespace for the turret (getting and shooting the ball)
namespace Turret {
constexpr int turretrotator =
    2;  // refers to the turret rotator (rotates turret)
constexpr int turretangler = 3;  // refers to the turret cowl (angles the
                                 // turret)
constexpr int shooterLeft = 8, shooterRight = 9;
constexpr double shooterSpdInit = 16400;
constexpr double rotMin = -149, rotMax = 134, elevMin = -3.4047,
                 elevMax = -0.01;
constexpr double rotatorpower = 0.3;  // this is the default power of the turret
                                      // rotator (default is positive)
constexpr double anglechangerpower =
    0.1;  // this is the power of the angle changer (default is positive)

}  // namespace Turret

// namespacefor the ball elevator
namespace ballelavator {

constexpr int ballelevator =
    4;  // refers to the belavator (stores balls as ammo before being ejected)
constexpr int intakenum = 4,
              elevnum = 5;  // DIO numbers for the infrared sensors

}  // namespace ballelavator
}  // namespace Constants