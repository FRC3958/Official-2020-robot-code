/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Legend:
     * MPS = meters per second
     * Native = native talon units, 4096 per rotation
     */

    public static final int kTimeout = 100;

    public static final int kQuadEncoderResolution = 4096; // same encoders are used everywhere, so this is ok
    public static final int kFalconResolution = 2048; // same encoders are used everywhere, so this is ok


    public static final class ControlConstants {

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kKeybindToggleIntake = Button.kBumperLeft.value;
        public static final int kKeybindShoot = Button.kA.value;
        public static final int kKeybindPrepareClimb = Button.kBack.value;
        public static final int kKeybindClimb = Button.kStart.value;
    }

    public static final class FieldConstants {

        public static final double kOuterPortCenterHeightMeters = Units.feetToMeters(98.25 / 12.0);
    }

    public static final class VisionConstants {

        // TODO: get actual measurements!!!
        public static final double kLimelightMountHeightMeters = Units.feetToMeters(24.0 / 12.0);
        public static final double kLimelightMountAngleDeg = 0.0;

        // TODO: tune
        public static final Gains kAlignToTargetGains = new Gains(0.0, 0.0, 0.0, 0.0);
        public static final double kAlignToTargetTolerancePosition = 0.05;
        public static final double kAlignToTargetToleranceVelocity = 0.05;
    }

    public static final class DriveConstants {

        public static final int kTalonPortFrontLeft = 1;
        public static final int kTalonPortFrontRight = 2;
        public static final int kTalonPortBackLeft = 3;
        public static final int kTalonPortBackRight = 4;

        // TODO: measure track width
        public static final double kTrackWidth = Units.feetToMeters(48.0 / 12.0);
        public static final double kWheelDiameterMeters = Units.feetToMeters(8.0/12.0);
        public static final double kWheelCircumferenceMeters = Math.PI * kWheelDiameterMeters;

        /**
         * Convert from meters to native untis
         * @param meters
         * @return
         */
        public static int getNativeFromMeters(double meters) {
            return (int)Math.round((meters / kWheelCircumferenceMeters) * kQuadEncoderResolution);
        }

        /**
         * Convert from native units to meters
         * @param nativeUnits
         * @return
         */
        public static double getMetersFromNative(int nativeUnits) {
            return ((double)nativeUnits / (double)kQuadEncoderResolution)
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

        public static final int kMaxVelocityNative = 20000;
        public static final double kMaxVelocityMPS = getVelocityMPSFromNative(kMaxVelocityNative);

        public static final double kMaxTurningVelocityRPS = Units.degreesToRadians(50.0);

        public static final int kPrimaryPIDLoopIdx = 0;

        public static final int kSlotVelocity = 0;

        // TODO: tune driving PIDs (again, since we are now using the quirky TalonSRX microprocessor)
        public static final Gains kGainsVelocity = new Gains(0.0, 0.0, 0.0, 0.0);
    }

    public static final class ShooterConstants {

        public static final int kTalonPortLeft = 5;
        public static final int kTalonPortRight = 6;

        // TODO: get actual measurements
        public static final double kShooterHeightMeters = Units.feetToMeters(24.0 / 12.0);
        public static final double kShooterAngleDeg = 30.0;

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

    public static final class IntakeConstants {

        public static final int kTalonPort = 7;

        public static final double kRunningPercentOutput = 0.7;

        // TODO: get these 2 values
        public static final int kCimChannel = 4;
        public static final double kEatenThreshold = 0.0;

        public static final int kSolenoidForwardChannel = 0;
        public static final int kSolenoidReverseChannel = 1;
    }

    public static final class SideBeltConstants {

        public static final int kTalonPort = 8;

        public static final double kRunningPercentOutput = 0.6;
    }

    public static final class ConveyorBeltConstants {

        public static final int kTalonPort = 9;

        public static final double kRunningPercentOutput = 0.5;
    }

    public static final class StopWheelConstants {

        public static final int kTalonPort = 10;

        public static final double kRunningPercentOutput = 0.3;
    }

    public static final class ClimberConstants {

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
}
