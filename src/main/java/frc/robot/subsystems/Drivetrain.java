/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  
  private final WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(DriveConstants.kTalonPortFrontLeft);
  private final WPI_TalonSRX m_leftSlave = new WPI_TalonSRX(DriveConstants.kTalonPortBackLeft);
  private final WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(DriveConstants.kTalonPortFrontRight);
  private final WPI_TalonSRX m_rightSlave = new WPI_TalonSRX(DriveConstants.kTalonPortBackRight);
  
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {

    /*
    m_leftMaster
    m_leftSlave
    m_rightMaster
    m_rightSlave
    */

    m_leftMaster.configFactoryDefault();
    m_leftSlave.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_rightSlave.configFactoryDefault();

    // brake for best control
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftSlave.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightSlave.setNeutralMode(NeutralMode.Brake);

    /**
     * 2 closed loop pids
     * id 0 (primary): average reading of both sides of drivetrain, used for distance & velocity
     * id 1 (aux): difference between both sides of drivetrain, used for turning
     */

    // config sensor for right, will be used as remote sensor
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0 /*irrelevant*/, Constants.kTimeout);

    // config remote talon's sensor (right talon's sensor) as remote sensor for the left talon
    m_leftMaster.configRemoteFeedbackFilter(
      m_rightMaster.getDeviceID(),
      RemoteSensorSource.TalonSRX_SelectedSensor, 
      1
    );

    // config sum to be used for velocity (we will avg them out later)
    m_leftMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.QuadEncoder);
    m_leftMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.RemoteSensor1);

    // config difference to be used for turn
    m_leftMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder);
    m_leftMaster.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1);

    // config sum to be used for velocity pid
    m_leftMaster.configSelectedFeedbackSensor(
      FeedbackDevice.SensorSum,
      DriveConstants.kDistancePIDLoopIdx,
      Constants.kTimeout
    );

    // must get half the sum, aka average, of both sides by setting the coeff to .5
    m_leftMaster.configSelectedFeedbackCoefficient(0.5, DriveConstants.kDistancePIDLoopIdx, Constants.kTimeout);

    // config difference to be used for turn pid
    m_leftMaster.configSelectedFeedbackSensor(
      FeedbackDevice.SensorDifference,
      DriveConstants.kTurnPIDLoopIdx,
      Constants.kTimeout
    );

    // config motor direction & sensor phases
    // TODO: actually get these
    m_leftMaster.setInverted(false);
    m_leftSlave.setInverted(false);
    m_leftMaster.setSensorPhase(true);

    m_rightMaster.setInverted(true);
    m_rightSlave.setInverted(true);
    m_rightMaster.setSensorPhase(true);

    
    /**
     * Gains config for both loops
     */
    m_leftMaster.config_kF(DriveConstants.kDistancePIDLoopIdx, DriveConstants.kDistanceGains.kF);
    m_leftMaster.config_kP(DriveConstants.kDistancePIDLoopIdx, DriveConstants.kDistanceGains.kP);
    m_leftMaster.config_kI(DriveConstants.kDistancePIDLoopIdx, DriveConstants.kDistanceGains.kI);
    m_leftMaster.config_kD(DriveConstants.kDistancePIDLoopIdx, DriveConstants.kDistanceGains.kD);

    m_leftMaster.config_kF(DriveConstants.kTurnPIDLoopIdx, DriveConstants.kTurnGains.kF);
    m_leftMaster.config_kP(DriveConstants.kTurnPIDLoopIdx, DriveConstants.kTurnGains.kP);
    m_leftMaster.config_kI(DriveConstants.kTurnPIDLoopIdx, DriveConstants.kTurnGains.kI);
    m_leftMaster.config_kD(DriveConstants.kTurnPIDLoopIdx, DriveConstants.kTurnGains.kD);

    m_leftSlave.follow(m_leftMaster);
    m_rightMaster.follow(m_leftMaster, FollowerType.AuxOutput1);
    m_rightSlave.follow(m_leftMaster, FollowerType.AuxOutput1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double forward, double turn) {
    m_leftMaster.set(ControlMode.Velocity)
  }
}
