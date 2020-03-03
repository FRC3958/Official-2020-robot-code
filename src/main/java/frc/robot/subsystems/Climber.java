/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
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

    TalonSRXConfiguration hookerConfig = new TalonSRXConfiguration();

    // encoder on hooker is used to deploy hook, then pull string back
    hookerConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

    // config gains
    hookerConfig.slot0.kF = ClimberConstants.kHookerPositionGains.kF;
    hookerConfig.slot0.kP = ClimberConstants.kHookerPositionGains.kP;
    hookerConfig.slot0.kI = ClimberConstants.kHookerPositionGains.kI;
    hookerConfig.slot0.kD = ClimberConstants.kHookerPositionGains.kD;

    // apply config
    m_hooker.configAllSettings(hookerConfig);

    TalonFXConfiguration winchConfig = new TalonFXConfiguration();

    // encoder on winch is used to lift to a desired height
    winchConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    // winch CANNOT turn backwards!!! zero any possible reverse values
    winchConfig.peakOutputReverse = 0.0;
    winchConfig.nominalOutputReverse = 0.0; 

    winchConfig.slot0.kF = ClimberConstants.kWinchPositionGains.kF;
    winchConfig.slot0.kP = ClimberConstants.kWinchPositionGains.kP;
    winchConfig.slot0.kI = ClimberConstants.kWinchPositionGains.kI;
    winchConfig.slot0.kD = ClimberConstants.kWinchPositionGains.kD;

    // apply config
    m_winch.configAllSettings(winchConfig);

    m_winch.setNeutralMode(NeutralMode.Brake);
    m_winch.setInverted(InvertType.None);

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
