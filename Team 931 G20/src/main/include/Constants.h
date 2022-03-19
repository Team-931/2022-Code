// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
# include <WPI/numbers>
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
    namespace DriveTrain
    {
        constexpr int drvnum[] {0,3,4,7}, trnnum[] {1,2,5,6};
        constexpr double halfLen = 29.25/2, halfWid = 19.75/2; // X is forward
        constexpr double offsetXs[] {halfLen, halfLen, -halfLen, -halfLen}; // coords in inches
        constexpr double offsetYs[] {halfWid, -halfWid, -halfWid, halfWid}; // right front is +, +
        constexpr double ticksPerRadian = 2048/2/wpi::numbers::pi;
    } // namespace DriveTrain
    
    namespace Intake {
        constexpr int whnum {1}, /* motor, they help suck the ball in */
        actnum {0}; /* mechanism that initiates intake mechanism*/
        constexpr double whpow = 0.75; //default power for the motor whnum
        


    }

    //namespace for the turret (getting and shooting the ball)
    namespace Turret{
        constexpr int turretrotator = 2; //refers to the turret rotator (rotates turret)
        constexpr int turretangler = 3; //refers to the turret cowl (angles the turret)
        
    }

    //namespacefor the ball elevator
    namespace ballelavator{

            constexpr int ballelevator = 4;  //refers to the belavator (stores balls as ammo before being ejected)


    }
}