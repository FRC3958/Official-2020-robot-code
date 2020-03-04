/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * Add your docs here.
 */
public final class ControlConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    /* For Non-Programmers: If you are trying to change the button of a mechanism,
     please refer to the Button.k(button).value. The Button can be replaced with a button of your choice
     You will also need to deploy the code, so press SHift+ Ctrl + p to do so. 
     When you do that, type deploy robot code and press it. 
    */

    public static class Driver {

        
    }

    public static class Operator {
        //Button used to take balls in

        public static final int kToggleIntake           = Button.kBumperLeft.value;

        // Button used to make the intake spin reverse

        public static final int kEjectBalls             = Button.kBumperRight.value;

        /* Button for Hooded Shooter to shoot sequentially,
        along with other commands like autoalign */

        public static final int kShoot                  = Button.kA.value;
        // Button to make the hook go up

        public static final int kPrepareClimb           = Button.kBack.value;

        // Button to get the gearbox to climb

        public static final int kClimb                  = Button.kStart.value;

        // Button to get the Limelight to switch between driver and vision mode
        
        public static final int kLimelightModeSwitch    = Button.kB.value;
    }
}

