/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  
  private final WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(DriveConstants.kTalonPortFrontLeft);
  private final WPI_TalonSRX m_leftSlave = new WPI_TalonSRX(DriveConstants.kTalonPortBackLeft);
  private final WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(DriveConstants.kTalonPortFrontRight);
  private final WPI_TalonSRX m_rightSlave = new WPI_TalonSRX(DriveConstants.kTalonPortBackRight);
  
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);
  private final DifferentialDriveOdometry m_odometry;

  private final AHRS m_ahrs = new AHRS(SPI.Port.kMXP);

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {

    m_leftMaster.configFactoryDefault();
    m_leftSlave.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_rightSlave.configFactoryDefault();

    // brake for best control
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftSlave.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightSlave.setNeutralMode(NeutralMode.Brake);

    // config sensor for right, will be used as remote sensor
    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, DriveConstants.kPrimaryPIDLoopIdx, Constants.kTimeout);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, DriveConstants.kPrimaryPIDLoopIdx, Constants.kTimeout);

    // config motor direction
    m_leftMaster.setInverted(false);
    m_leftSlave.setInverted(false);
    m_rightMaster.setInverted(true);
    m_rightSlave.setInverted(true);

    // sensor phases
    m_leftMaster.setSensorPhase(true);
    m_rightMaster.setSensorPhase(true);
    
    /**
     * Gains config for both loops
     */
    m_leftMaster.config_kF(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kF);
    m_leftMaster.config_kP(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kP);
    m_leftMaster.config_kI(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kI); 
    m_leftMaster.config_kD(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kD);
    m_rightMaster.config_kF(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kF);
    m_rightMaster.config_kP(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kP);
    m_rightMaster.config_kI(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kI); 
    m_rightMaster.config_kD(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kD);

    // follow masters
    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    // reset measurements
    resetEncoders();
    resetHeading();

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Serve odometry updates
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()), 
      getLeftDistanceMeters(),
      getRightDistanceMeters()
    );
  }

  /**
   * Drive arcade style
   * @param forward -1.0 to +1.0 value indicating backward (neg) or forward (pos)
   * @param turn -1.0 to +1.0 indicating left (neg) or right (pos)
   */
  public void arcadeDrive(double forward, double turn) {

    chassisSpeedsDrive(
      new ChassisSpeeds(
        forward * DriveConstants.kMaxVelocityMPS,
        0.0,
        turn * DriveConstants.kMaxTurningVelocityRPS)
    );
  }

  /**
   * Set velocity targets according to chassis speeds
   * @param speeds
   */
  public void chassisSpeedsDrive(ChassisSpeeds speeds) {

    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

    m_leftMaster.set(ControlMode.Velocity, 
      DriveConstants.getVelocityNativeFromMPS(wheelSpeeds.leftMetersPerSecond)
    );

    m_rightMaster.set(ControlMode.Velocity, 
      DriveConstants.getVelocityNativeFromMPS(wheelSpeeds.rightMetersPerSecond)
    );
  }

  public double getLeftDistanceMeters() {
    return DriveConstants.getMetersFromNative(m_leftMaster.getSelectedSensorPosition());
  }

  public double getRightDistanceMeters() {
    return DriveConstants.getMetersFromNative(m_rightMaster.getSelectedSensorPosition());
  }

  public void resetEncoders() {
    m_leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeout);
    m_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeout);
  }

  /**
   * Gets the angle of the bot
   * @return
   */
  public double getHeading() {
    return m_ahrs.getAngle();
  }

  public void resetHeading() {
    m_ahrs.zeroYaw();
  }
}
