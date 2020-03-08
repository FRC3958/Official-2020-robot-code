/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  
  private final WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(DriveConstants.kTalonPortBackLeft);
  private final WPI_TalonSRX m_leftSlave = new WPI_TalonSRX(DriveConstants.kTalonPortFrontLeft);
  private final WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(DriveConstants.kTalonPortBackRight);
  private final WPI_TalonSRX m_rightSlave = new WPI_TalonSRX(DriveConstants.kTalonPortFrontRight);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

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

    // so that talons will show forward indicator
    m_drive.setRightSideInverted(false);

    // sensor phases
    m_leftMaster.setSensorPhase(false);
    m_rightMaster.setSensorPhase(true);

    // config motor direction    
    m_leftMaster.setInverted(InvertType.None);
    m_leftSlave.setInverted(InvertType.None);
    m_rightMaster.setInverted(InvertType.InvertMotorOutput);
    m_rightSlave.setInverted(InvertType.None);

    // follow masters
    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    // current limiting
    final int peakAmps = 80;
    m_leftMaster.configPeakCurrentLimit(peakAmps);
    m_leftSlave.configPeakCurrentLimit(peakAmps);
    m_rightMaster.configPeakCurrentLimit(peakAmps);
    m_rightSlave.configPeakCurrentLimit(peakAmps);

    final int peakTime = 0;
    m_leftMaster.configPeakCurrentDuration(peakTime);
    m_leftSlave.configPeakCurrentDuration(peakTime);
    m_rightMaster.configPeakCurrentDuration(peakTime);
    m_rightSlave.configPeakCurrentDuration(peakTime);

    final int continousAmps = 60;
    m_leftMaster.configContinuousCurrentLimit(continousAmps);
    m_leftSlave.configContinuousCurrentLimit(continousAmps);
    m_rightMaster.configContinuousCurrentLimit(continousAmps);
    m_rightSlave.configContinuousCurrentLimit(continousAmps);

    m_leftMaster.enableCurrentLimit(true);
    m_leftSlave.enableCurrentLimit(true);
    m_rightMaster.enableCurrentLimit(true);
    m_rightSlave.enableCurrentLimit(true);

    // reset measurements
    resetEncoders();
    resetHeading();

    // instantiate odometry object after resetting everything
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

    updateSmartDashboard();
  }

  private void updateSmartDashboard() {

    var speeds = getWheelSpeeds();

    SmartDashboard.putNumber("DT Left m/s", speeds.leftMetersPerSecond);
    SmartDashboard.putNumber("DT Right m/s", speeds.rightMetersPerSecond);
    SmartDashboard.putNumber("DT velocity m/s", (speeds.leftMetersPerSecond + speeds.rightMetersPerSecond) * .5);
    SmartDashboard.putNumber("Distance left", getLeftDistanceMeters());
    SmartDashboard.putNumber("Distance right", getRightDistanceMeters());    
  }

  /**
   * Drive arcade style
   * @param forward -1.0 to +1.0 value indicating backward (neg) or forward (pos)
   * @param turn -1.0 to +1.0 indicating left (neg) or right (pos)
   */
  public void arcadeDrive(double forward, double turn) {
    // m_drive.arcadeDrive(forward, turn, false);
  }

  public void tankDriveVolts(double leftVoltage, double rightVoltage) {

    // m_leftMaster.setVoltage(leftVoltage);
    // m_rightMaster.setVoltage(-rightVoltage);

    m_drive.feed();
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
    return -m_ahrs.getAngle();
  }

  public void resetHeading() {
    m_ahrs.zeroYaw();
  }
}
