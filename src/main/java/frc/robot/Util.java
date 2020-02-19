/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Util {

    public static double deadband(double value, double deadband) {
        return (Math.abs(value) < deadband) ? 0.0 : value;
    }

    // TODO: figure out best way to solve for RPM from distane
    public static double calculateRPM(double targetDistance) {

        return 7700.0 / (targetDistance / 25.0);
    }
}
