/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
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

        public static final int kForwardDrive           = Axis.kRightY.value;
        public static final int kTurnDrive              = Axis.kLeftX.value;

        public static final int kAutoAlign              = Button.kBumperLeft.value;
    }

    public static class Operator {

        //Button used to take balls in
        public static final int kEat                    = Button.kB.value;
        
        // Button used to make the intake spin reverse
        public static final int kEjectBalls             = Button.kY.value;

        // Lift or lower intake (toggle)
        public static final int kLiftLowerIntake        = Button.kA.value;

        // Trigger for manual ramp up
        public static final int kManualRampUp           = Axis.kLeftTrigger.value;

        // Trigger to shoot
        public static final int kShoot                  = Axis.kRightTrigger.value;

        // Indexer
        public static final int kClearIndexer           = Axis.kLeftX.value;

        // Raise/lower shaft
        public static final int kToggleClimberPiston    = Button.kBack.value;

        // Extand shaft
        public static final int kExtendShaft            = Button.kBumperRight.value;

        // Retract shaft
        public static final int kRetractShaft           = Button.kBumperLeft.value;

        // Lift (with winch)
        public static final int kLift                   = Axis.kRightY.value;

        // Button to get the Limelight to switch between driver and vision mode
        public static final int kLimelightModeSwitch    = Button.kStart.value;
    }
}

