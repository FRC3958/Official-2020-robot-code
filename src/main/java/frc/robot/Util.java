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
    public static double calculateRPM(double targetDistanceMeters) {
        
        double x = targetDistanceMeters;

        // 3.6815008x^5 - 115.9117541x^4 + 1349.00234x^3 - 6908.883255x^2 + 13561.41364x + 1388.348389

        return (3.6815008 * Math.pow(x, 5)) + (-115.9117541 * Math.pow(x, 4)) + (1349.00234 * Math.pow(x, 3)) 
        + (-6908.883255 * Math.pow(x, 2)) + (13561.41364 * Math.pow(x, 1)) +  1388.348389;


        /**
        3.6815008
        - 115.9117541
        1349.00234
        - 6908.883255
        13561.41364
        1388.348389
         */

        /**
        * Math.pow(x, 5) +
        * Math.pow(x, 4)
        * Math.pow(x, 3)
        * Math.pow(x, 2)
        * Math.pow(x, 1)
        * x
         */
    }
}
