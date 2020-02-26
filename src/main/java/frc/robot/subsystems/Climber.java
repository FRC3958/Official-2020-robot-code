/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private final WPI_TalonSRX m_hooker = new WPI_TalonSRX(ClimberConstants.kTalonPortHooker);
  private final WPI_TalonFX m_winch = new WPI_TalonFX(ClimberConstants.kTalonPortLifter);

  private final Solenoid m_piston = new Solenoid(ClimberConstants.kPCMIdPiston);

  /**
   * Creates a new Climber.
   */
  public Climber() {

    // encoder on hooker is used to deploy hook, then pull string back
    m_hooker.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    m_hooker.config_kF(ClimberConstants.kPrimaryPIDLoopIdx, ClimberConstants.kHookerPositionGains.kF);
    m_hooker.config_kP(ClimberConstants.kPrimaryPIDLoopIdx, ClimberConstants.kHookerPositionGains.kP);
    m_hooker.config_kI(ClimberConstants.kPrimaryPIDLoopIdx, ClimberConstants.kHookerPositionGains.kI);
    m_hooker.config_kD(ClimberConstants.kPrimaryPIDLoopIdx, ClimberConstants.kHookerPositionGains.kD);

    // encoder on winch is used to lift to a desired height
    m_winch.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    // winch CANNOT turn backwards!!! zero any possible reverse values
    m_winch.configNominalOutputReverse(0.0);
    m_winch.configPeakOutputReverse(0.0);

    m_winch.config_kF(ClimberConstants.kPrimaryPIDLoopIdx, ClimberConstants.kWinchPositionGains.kF);
    m_winch.config_kP(ClimberConstants.kPrimaryPIDLoopIdx, ClimberConstants.kWinchPositionGains.kP);
    m_winch.config_kI(ClimberConstants.kPrimaryPIDLoopIdx, ClimberConstants.kWinchPositionGains.kI);
    m_winch.config_kD(ClimberConstants.kPrimaryPIDLoopIdx, ClimberConstants.kWinchPositionGains.kD);

    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void raiseHook() {

    m_piston.set(true);
  }

  public void lowerHook() {

    m_piston.set(false);
  }

  public void deployHook() {

    m_hooker.set(ControlMode.Position, ClimberConstants.kHookerDeployedPosition);
  }

  public void retractHook() {

    m_hooker.set(ControlMode.Position, 0);
  }

  public boolean isHookDeployed() {

    return ((Math.abs(ClimberConstants.kHookerDeployedPosition - m_hooker.getSelectedSensorPosition())) 
      / ClimberConstants.kHookerDeployedPosition) <= ClimberConstants.kHookerDeployedPercentTolerance;
  }

  public boolean isHookRetracted() {

    return ((Math.abs(ClimberConstants.kHookerDeployedPosition - m_hooker.getSelectedSensorPosition())) 
      / ClimberConstants.kHookerDeployedPosition) >= (1.0 - ClimberConstants.kHookerDeployedPercentTolerance);
  }

  public void lift() {

    m_winch.set(ControlMode.Position, ClimberConstants.kWinchLiftedPosition);
  }

  public void stopLifting() {

    m_winch.set(ControlMode.Position, m_winch.getSelectedSensorPosition());
  }

  public boolean isLiftedToHeight() {

    return ((Math.abs(m_winch.getSelectedSensorPosition()) - ClimberConstants.kWinchLiftedPosition) 
      / ClimberConstants.kWinchLiftedPosition) <= ClimberConstants.kWinchLiftedPercentTolerance;
  }

  private void resetEncoders() {

    m_hooker.getSensorCollection().setQuadraturePosition(0, Constants.kTimeout);
    m_winch.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeout);
  }
}
