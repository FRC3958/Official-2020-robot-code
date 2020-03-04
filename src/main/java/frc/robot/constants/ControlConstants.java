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

    public static class Driver {

        
    }

    public static class Operator {

        public static final int kToggleIntake           = Button.kBumperLeft.value;
        public static final int kEjectBalls             = Button.kBumperRight.value;

        public static final int kShoot                  = Button.kA.value;

        public static final int kPrepareClimb           = Button.kBack.value;
        public static final int kClimb                  = Button.kStart.value;
        
        public static final int kLimelightModeSwitch    = Button.kB.value;
    }
}

