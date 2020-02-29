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
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  
  private final WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(DriveConstants.kTalonPortFrontLeft);
  private final WPI_TalonSRX m_leftSlave = new WPI_TalonSRX(DriveConstants.kTalonPortBackLeft);
  private final WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(DriveConstants.kTalonPortFrontRight);
  private final WPI_TalonSRX m_rightSlave = new WPI_TalonSRX(DriveConstants.kTalonPortBackRight);

  private final DifferentialDriveOdometry m_odometry;

  private final AHRS m_ahrs = new AHRS(SPI.Port.kMXP);

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {

    TalonSRXConfiguration leftConfig = new TalonSRXConfiguration();
    TalonSRXConfiguration rightConfig = new TalonSRXConfiguration();

    // config sensor for right, will be used as remote sensor
    leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    rightConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    
    /**
     * Gains config for both loops
     */
    leftConfig.slot0.kF = DriveConstants.kGainsVelocity.kF;
    leftConfig.slot0.kP = DriveConstants.kGainsVelocity.kP;
    leftConfig.slot0.kI = DriveConstants.kGainsVelocity.kI;
    leftConfig.slot0.kD = DriveConstants.kGainsVelocity.kD;

    rightConfig.slot0.kF = DriveConstants.kGainsVelocity.kF;
    rightConfig.slot0.kP = DriveConstants.kGainsVelocity.kP;
    rightConfig.slot0.kI = DriveConstants.kGainsVelocity.kI;
    rightConfig.slot0.kD = DriveConstants.kGainsVelocity.kD;

    // apply configs
    m_leftMaster.configAllSettings(leftConfig);
    m_rightMaster.configAllSettings(rightConfig);
    m_leftSlave.configFactoryDefault();
    m_rightSlave.configFactoryDefault();

    // brake for best control
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftSlave.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightSlave.setNeutralMode(NeutralMode.Brake);

    // config motor direction
    m_leftMaster.setInverted(false);
    m_leftSlave.setInverted(false);
    m_rightMaster.setInverted(true);
    m_rightSlave.setInverted(true);

    // sensor phases
    m_leftMaster.setSensorPhase(true);
    m_rightMaster.setSensorPhase(true);

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

    DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kKinematics.toWheelSpeeds(speeds);

    m_leftMaster.set(ControlMode.Velocity, 
      DriveConstants.getVelocityNativeFromMPS(wheelSpeeds.leftMetersPerSecond)
    );

    m_rightMaster.set(ControlMode.Velocity, 
      DriveConstants.getVelocityNativeFromMPS(wheelSpeeds.rightMetersPerSecond)
    );
  }

  public void tankDriveVolts(double leftVoltage, double rightVoltage) {

    m_leftMaster.setVoltage(-leftVoltage);
    m_rightMaster.setVoltage(+rightVoltage);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(
      DriveConstants.getVelocityMPSFromNative(m_leftMaster.getSelectedSensorVelocity()),
      DriveConstants.getVelocityMPSFromNative(m_rightMaster.getSelectedSensorVelocity())
    );
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
