/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Add your docs here.
 */
public final class DriveConstants {

    public static final int kTalonPortFrontLeft = 1;
    public static final int kTalonPortFrontRight = 2;
    public static final int kTalonPortBackLeft = 3;
    public static final int kTalonPortBackRight = 4;

    public static final double kTrackWidth = Units.feetToMeters(21.2 / 12.0);
    public static final double kWheelDiameterMeters = Units.feetToMeters(8.0/12.0);
    public static final double kWheelCircumferenceMeters = Math.PI * kWheelDiameterMeters;
    
    public static final DifferentialDriveKinematics kKinematics = new DifferentialDriveKinematics(kTrackWidth);

    /**
     * Convert from meters to native untis
     * @param meters
     * @return
     */
    public static int getNativeFromMeters(double meters) {
        return (int)Math.round((meters / kWheelCircumferenceMeters) * Constants.kQuadEncoderResolution);
    }

    /**
     * Convert from native units to meters
     * @param nativeUnits
     * @return
     */
    public static double getMetersFromNative(int nativeUnits) {
        return ((double)nativeUnits / (double)Constants.kQuadEncoderResolution)
            * kWheelCircumferenceMeters;
    }

    /**
     * Convert from meters per second to native units per 100ms
     * @param mps
     * @return
     */
    public static int getVelocityNativeFromMPS(double mps) {
        return (int)Math.round((double)getNativeFromMeters(mps) / 10.0);
    }

    /**
     * Convert from native units per 100ms to meters per second
     * @param velocityNative
     * @return
     */
    public static double getVelocityMPSFromNative(int velocityNative) {
        return getMetersFromNative(velocityNative) * 10.0;
    }
}
