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

    //https://docs.google.com/spreadsheets/d/1oIpPNEuoaOrB24wLa9FMBx6Za_Jd8xJV_Y1EI6LmxWM/edit#gid=0
    public static double calculateRPM(double targetDistance) {
        
        return 4297 - (433 * targetDistance) + (175 * Math.pow(targetDistance, 2));
    }
}
