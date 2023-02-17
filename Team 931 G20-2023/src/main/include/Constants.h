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
constexpr int absSubtraction[]{2220, 2148, 2695, 1023};  // to align the wheels
constexpr double maxVel = 1500, maxAccel = 1500;//for Motion Magic
constexpr double CtlP = 0.1, CtlF = 0.3;//for PID
}  // namespace DriveTrain

namespace Intake {
constexpr int whnum{1},         /* motor, they help suck the ball in */
    raisenum{4}, lownum{0};     /* mechanism that initiates intake mechanism*/
constexpr double whpow = 1.00;  // default power for the motor whnum

}  // namespace Intake

// namespace for the arm (getting and shooting the ball)
namespace Arm {
constexpr int stage1Id = 8, stage2Id = 9;

constexpr double maxVel = 1500, maxAccel = 1500;//for Motion Magic
constexpr double CtlP = 0.1, CtlF = 0.3;//for PID

}  // namespace Arm

}  // namespace Constants