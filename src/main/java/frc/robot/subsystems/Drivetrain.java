/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

// if you are confused, look here. then look at ReadTheDocs ctre!!!
// https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/VelocityClosedLoop_AuxStraightQuadrature/src/main/java/frc/robot/Robot.java

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

    // // /**
    // //  * 2 closed loop pids
    // //  * id 0 (primary): average reading of both sides of drivetrain, used for distance & velocity
    // //  * id 1 (aux): difference between both sides of drivetrain, used for turning
    // //  */

    // // // config sensor for right, will be used as remote sensor
    // // m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0 /*irrelevant*/, Constants.kTimeout);

    // // // config remote talon's sensor (right talon's sensor) as remote sensor for the left talon
    // // m_leftMaster.configRemoteFeedbackFilter(
    // //   m_rightMaster.getDeviceID(),
    // //   RemoteSensorSource.TalonSRX_SelectedSensor, 
    // //   1
    // // );

    // // // config sum to be used for velocity (we will avg them out later)
    // // m_leftMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1);
    // // m_leftMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative);

    // // // config difference to be used for turn
    // // m_leftMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative);
    // // m_leftMaster.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1);

    // // // config sum to be used for velocity pid (they mult by .5, so effectively an avg)
    // // m_leftMaster.configSelectedFeedbackSensor(
    // //   FeedbackDevice.SensorSum,
    // //   DriveConstants.kPrimaryPIDLoopIdx,
    // //   Constants.kTimeout
    // // );

    // // // must get half the sum, aka average, of both sides by setting the coeff to .5
    // // m_leftMaster.configSelectedFeedbackCoefficient(0.5, DriveConstants.kPrimaryPIDLoopIdx, Constants.kTimeout);

    // // // config difference to be used for turn pid
    // // m_leftMaster.configSelectedFeedbackSensor(
    // //   FeedbackDevice.SensorDifference,
    // //   DriveConstants.kTurnPIDLoopIdx,
    // //   Constants.kTimeout
    // // );

    // // // config motor direction & sensor phases
    // // m_leftMaster.setInverted(false);
    // // m_leftSlave.setInverted(false);
    // // m_leftMaster.setSensorPhase(false);

    // // m_rightMaster.setInverted(true);
    // // m_rightSlave.setInverted(true);
    // // m_rightMaster.setSensorPhase(true);
    
    // // /**
    // //  * Gains config for both loops
    // //  */
    // // m_leftMaster.config_kF(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kF);
    // // m_leftMaster.config_kP(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kP);
    // // m_leftMaster.config_kI(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kI); 
    // // m_leftMaster.config_kD(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kD);

    // // m_leftMaster.config_kF(DriveConstants.kSlotTurning, DriveConstants.kGainsTurn.kF);
    // // m_leftMaster.config_kP(DriveConstants.kSlotTurning, DriveConstants.kGainsTurn.kP);
    // // m_leftMaster.config_kI(DriveConstants.kSlotTurning, DriveConstants.kGainsTurn.kI);
    // // m_leftMaster.config_kD(DriveConstants.kSlotTurning, DriveConstants.kGainsTurn.kD);

    // // // follow masters. make rightMaster follow leftMaster AuxOutput follow mode when using the turn pid
    // // // do that after setting the leftMaster... not anywhere else
    // // m_leftSlave.follow(m_leftMaster);
    // // m_rightSlave.follow(m_rightMaster);

    /**
     * 2 closed loop pids
     * id 0 (primary): average reading of both sides of drivetrain, used for distance & velocity
     * id 1 (aux): difference between both sides of drivetrain, used for turning
     */

    // config sensor for right, will be used as remote sensor
    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0 /*irrelevant*/, Constants.kTimeout);

    // config remote talon's sensor (right talon's sensor) as remote sensor for the left talon
    m_rightMaster.configRemoteFeedbackFilter(
      m_leftMaster.getDeviceID(),
      RemoteSensorSource.TalonSRX_SelectedSensor, 
      1
    );

    // config sum to be used for velocity (we will avg them out later)
    m_rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1);
    m_rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative);

    // config difference to be used for turn
    m_rightMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative);
    m_rightMaster.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1);

    // config sum to be used for velocity pid (they mult by .5, so effectively an avg)
    m_rightMaster.configSelectedFeedbackSensor(
      FeedbackDevice.SensorSum,
      DriveConstants.kPrimaryPIDLoopIdx,
      Constants.kTimeout
    );

    // must get half the sum, aka average, of both sides by setting the coeff to .5
    m_rightMaster.configSelectedFeedbackCoefficient(0.5, DriveConstants.kPrimaryPIDLoopIdx, Constants.kTimeout);

    // config difference to be used for turn pid
    m_rightMaster.configSelectedFeedbackSensor(
      FeedbackDevice.SensorDifference,
      DriveConstants.kTurnPIDLoopIdx,
      Constants.kTimeout
    );

    // config motor direction & sensor phases
    m_leftMaster.setInverted(false);
    m_leftSlave.setInverted(false);
    m_rightMaster.setInverted(true);
    m_rightSlave.setInverted(true);

    m_leftMaster.setSensorPhase(true);
    m_rightMaster.setSensorPhase(true);
    
    /**
     * Gains config for both loops
     */
    m_rightMaster.config_kF(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kF);
    m_rightMaster.config_kP(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kP);
    m_rightMaster.config_kI(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kI); 
    m_rightMaster.config_kD(DriveConstants.kSlotVelocity, DriveConstants.kGainsVelocity.kD);

    m_rightMaster.config_kF(DriveConstants.kSlotTurning, DriveConstants.kGainsTurn.kF);
    m_rightMaster.config_kP(DriveConstants.kSlotTurning, DriveConstants.kGainsTurn.kP);
    m_rightMaster.config_kI(DriveConstants.kSlotTurning, DriveConstants.kGainsTurn.kI);
    m_rightMaster.config_kD(DriveConstants.kSlotTurning, DriveConstants.kGainsTurn.kD);

    // follow masters. make rightMaster follow leftMaster AuxOutput follow mode when using the turn pid
    // do that after setting the leftMaster... not anywhere else
    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    m_rightMaster.follow(m_leftMaster, FollowerType.AuxOutput1);

    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /**
     * Encoder info
     */
    int leftNative = -m_leftMaster.getSensorCollection().getQuadratureVelocity();
    int rightNative = m_rightMaster.getSensorCollection().getQuadratureVelocity();

    SmartDashboard.putNumber("left native", leftNative);
    SmartDashboard.putNumber("right native", rightNative);

    SmartDashboard.putNumber("left mps", DriveConstants.getVelocityMPS(leftNative));
    SmartDashboard.putNumber("right mps", DriveConstants.getVelocityMPS(rightNative));

    /**
     * PID info
     */
    SmartDashboard.putNumber("difference heading", getDifferenceHeading());
  }

  /**
   * Drive arcade style
   * @param forward -1.0 to +1.0 value indicating backward (neg) or forward (pos)
   * @param turn -1.0 to +1.0 indicating left (neg) or right (pos)
   */
  public void arcadeDrive(double forward, double turn) {

    forward = MathUtil.clamp(forward, -1.0, +1.0);
    turn = MathUtil.clamp(turn, -1.0, +1.0);

    m_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
    m_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
  }

  /**
   * Gets the difference heading (difference between both sides' positions) for use with driveStraight
   * @return
   */
  public int getDifferenceHeading() {
    
    return m_leftMaster.getSelectedSensorPosition(DriveConstants.kTurnPIDLoopIdx);
  }

  /**
   * Drive straight
   * @param forward -1.0 to +1.0 value indicating backward (neg) or forward (pos)
   * @param setpointHeading Position difference gotten from the turn loop (use getDifferenceHeading)
   */
  public void driveStraight(double forward, int setpointHeading) {
    
    forward = MathUtil.clamp(forward, -1.0, +1.0) * DriveConstants.kMaxVelocityNative;

    m_leftMaster.set(ControlMode.Velocity, forward, DemandType.AuxPID, setpointHeading);
    m_rightMaster.follow(m_leftMaster, FollowerType.AuxOutput1);
  }

  public void resetEncoders() {
    m_leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeout);
    m_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeout);
  }
}
