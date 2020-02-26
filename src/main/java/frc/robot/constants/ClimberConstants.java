/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import frc.robot.Gains;

/**
 * Add your docs here.
 */
public final class ClimberConstants {

    public static final int kTalonPortHooker = 11;
    public static final int kTalonPortLifter = 12;

    public static final int kPrimaryPIDLoopIdx = 0;

    public static final Gains kHookerPositionGains = new Gains(0.0, 0.0, 0.0, 0.0);

    // TODO: replace placeholder
    public static final int kHookerDeployedPosition = Constants.kQuadEncoderResolution * 50;
    public static final double kHookerDeployedPercentTolerance = 0.10;

    public static final Gains kWinchPositionGains = new Gains(0.0, 0.0, 0.0, 0.0);

    // TODO: replace placeholder
    public static final int kWinchLiftedPosition = Constants.kQuadEncoderResolution * 50;
    public static final double kWinchLiftedPercentTolerance = 0.10;
}