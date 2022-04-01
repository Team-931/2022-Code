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
 * 
 * 
 * 
 */


//HERE ARE MOTOR POWERS BASED ON BALLEVATOR STATES
const double BALLEVATOR_SPEED_IDLE = 0.0;
const double BALLEVATOR_SPEED_READY = 0.75;
const double BALLEVATOR_SPEED_LOADING = 0.65;
const double BALLEVATOR_SPEED_HOLD = 0.0;
const double BALLEVATOR_SPEED_FIRE = 1.0;
const double BALLEVATOR_SPEED_REVERSE = -1.0;

const double TURRET_YAW_DEADZONE = 0.25;
const double TURRET_ANGLE_DEADZONE = 0.2;
const double TURRET_SPEED_DEADZONE = 0.2;

#define DBL_MAX std::numeric_limits<double>::max()
const std::vector<double> auto_pitch_ty = {-DBL_MAX, -20, 
                                           -4.9,    15.8,   DBL_MAX};
const std::vector<double> auto_pitch_speed = {15500, 13250, 
                                              11250, 11000, 11000};
const std::vector<double> auto_pitch_angle = {25.48, 18.81,
                                              15.93, 10.48, 10.48};

namespace Constants {
namespace RobotContainer {
constexpr double minThrottle = .15,
                 maxThrottle = .8;
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
    ticksPerAbsTick = turnGearing * 2048,
    wheelDiameter = 4, // inches
    wheelGearRatio = 6.54,
    ticksPerInch = 2048 * wheelGearRatio / wpi::numbers::pi / wheelDiameter;
constexpr int absSubtraction[]{3265, 2068, 135, 2673};  // to align the wheels
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
constexpr double rotMin = -50, rotMax = 50;
constexpr double elevMin = 8.0, elevMax = 120;//ANGLE CHANGE LIMITS

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