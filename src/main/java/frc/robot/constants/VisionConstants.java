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

    public static final double kLimelightMountHeightMeters = Units.feetToMeters((15.0 /*from base*/ + 6.5 /*ground to base*/) / 12.0);
    public static final double kLimelightMountAngleDeg = 12.0;
    public static final double kLimelightMountDistanceFromBackMeters = Units.feetToMeters(22.0 / 12.0);

    // TODO: tune
    public static final Gains kAlignToTargetGains = new Gains(0.0, 0.15, 0.33, 0.05);
    public static final double kAlignToTargetTolerancePosition = 0.05;
    public static final double kAlignToTargetToleranceVelocity = 0.05;

    // from limelight wiki
    public static class LedMode {

        public static final int kPipeline = 0;
        public static final int kForceOff = 1;
        public static final int kForceBlink = 2;
        public static final int kForceOn = 3;
    }

    // from limelight wiki
    public static class CamMode {

        public static final int kVisionProcessor = 0;
        public static final int kDriver = 1;
    }

    // custom
    public static class Pipeline {

        public static final int kTargetPipeline = 0;
        public static final int kDriverPipeline = 1;
    }
}
