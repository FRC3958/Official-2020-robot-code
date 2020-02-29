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
public final class ShooterConstants {

    public static final int kTalonPortLeft = 5;
    public static final int kTalonPortRight = 6;

    public static final double kShooterHeightMeters = Units.feetToMeters(26.0 / 12.0); // center of ball
    public static final double kShooterAngleDeg = 26.5;

    /**
     * Convert from native units per 100ms to rotations per minute
     * @param rpm
     * @return
     */
    public static int getVelocityNativeFromRPM(double rpm) {

        return (int)Math.round((rpm / 600.0) * Constants.kQuadEncoderResolution);
    }

    public static double getRPMFromNativeVelocity(int velocity) {
        return (double)velocity * 600.0 / Constants.kQuadEncoderResolution;
    }

    public static final int kPIDLoopIdx = 0;

    public static final Gains kGains = new Gains(0.03, 0.45, 0.0, 0.0);

    public static final double kAcceptablePercentError = 0.02;

    public static final double kShootDipPercent = 0.10;
    public static final double kTimeToShoot = 0.5;
}
