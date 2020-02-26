/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Gains;

/**
 * Add your docs here.
 */

public final class VisionConstants {

    // TODO: get actual measurements!!!
    public static final double kLimelightMountHeightMeters = Units.feetToMeters(24.0 / 12.0);
    public static final double kLimelightMountAngleDeg = 0.0;

    // TODO: tune
    public static final Gains kAlignToTargetGains = new Gains(0.0, 0.0, 0.0, 0.0);
    public static final double kAlignToTargetTolerancePosition = 0.05;
    public static final double kAlignToTargetToleranceVelocity = 0.05;
}