// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <numbers>
#include <cmath>
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 * 
 * 
 * 
 */



const double BALLEVATOR_SPEED_IDLE = 0.0;
const double BALLEVATOR_SPEED_READY = 0.75;
const double BALLEVATOR_SPEED_LOADING = 0.65;
const double BALLEVATOR_SPEED_HOLD = 0.0;
const double BALLEVATOR_SPEED_FIRE = 1.0;
const double BALLEVATOR_SPEED_REVERSE = -1.0;

const double TURRET_YAW_DEADZONE = 0.25;
const double TURRET_ANGLE_DEADZONE = 0.2;
const double TURRET_SPEED_DEADZONE = 0.2;

namespace Constants {
namespace RobotContainer {
constexpr double minThrottle = .1;
}
namespace DriveTrain {
constexpr int drvnum[]{1, 4, 7, 2}, trnnum[]{0, 5, 6, 3},
    encodernum[]{0, 3, 2, 1};
constexpr double halfLen = 24.25 / 2, halfWid = 24.125 / 2;  // X is forward
constexpr double offsetXs[]{halfLen, halfLen, -halfLen,
                            -halfLen};  // coords in inches
constexpr double offsetYs[]{halfWid, -halfWid, -halfWid,
                            halfWid};  // right front is +, +
const double rotationRescale = std::sqrt(
    halfLen * halfLen +
    halfWid * halfWid);  // todo: de-kludge this, it makes the linear and
                         // rotational control argumenrs comparable.
constexpr double turnGearing = 72.0 / 14 * 24 / 12,  // maybe use std::ratio
    ticksPerRadian = 2048 / 2 / std::numbers::pi * turnGearing,
                 ticksPerAbsTick =
                     turnGearing *
                     2048 /*/ 4096*/;  // todo: check this with hardware
constexpr int absSubtraction[]{-44, 75+2048, 2695, 1033};  // to align the wheels
}  // namespace DriveTrain

namespace Intake {
constexpr int whnum{1},         /* motor, they help suck the ball in */
    raisenum{4}, lownum{0};     /* mechanism that initiates intake mechanism*/
constexpr double whpow = 1.00;  // default power for the motor whnum

}  // namespace Intake

// namespace for the turret (getting and shooting the ball)
namespace Turret {
constexpr int turretrotator =
    2;  // refers to the turret rotator (rotates turret)
constexpr int turretangler = 3;  // refers to the turret cowl (angles the
                                 // turret)
constexpr int shooterLeft = 8, shooterRight = 9;

constexpr double shooterSpdInit = 11000;
constexpr double rotMin = -125, rotMax = 110;
constexpr double elevMin = 8.0, elevMax = 120;

// Scales the rate of adjustment to turret rotation and
// angle changer positions.
constexpr double rotatorpower = 0.3;
constexpr double anglechangerpower = 1.0;

constexpr double shooterCtlP = 0.1, shooterCtlI = 0.0001;
constexpr double elevCtlP = 0.05, elevCtlI = 0.0;

}  // namespace Turret

// namespacefor the ball elevator
namespace ballelavator {

constexpr int ballelevator =
    4;  // refers to the belavator (stores balls as ammo before being ejected)
constexpr int intakenum = 4,
              elevnum = 5;  // DIO numbers for the infrared sensors

}  // namespace ballelavator
}  // namespace Constants