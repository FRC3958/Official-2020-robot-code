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
public class Gains {

    public double kF, kP, kI, kD;

    public Gains(double f, double p, double i, double d) {
        kF = f;
        kP = p;
        kI = i;
        kD = d;
    }
}
