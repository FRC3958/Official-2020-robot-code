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
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {

  private final WPI_TalonSRX m_hooker = new WPI_TalonSRX(ClimberConstants.kTalonPortHooker);
  private final WPI_TalonFX m_winch = new WPI_TalonFX(ClimberConstants.kTalonPortLifter);

  private final DoubleSolenoid m_piston = new DoubleSolenoid(ClimberConstants.kPCMPistonForward, ClimberConstants.kPCMPistonReverse);

  private boolean m_hasHookDeployed;

  private final Orchestra m_orchestra = new Orchestra();

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

    // apply config
    m_winch.configAllSettings(winchConfig);

    m_winch.setNeutralMode(NeutralMode.Brake);
    m_winch.setInverted(InvertType.None);

    resetEncoders();

    m_hasHookDeployed = false;

    /**
     * Playing music yay
     */
    m_orchestra.loadMusic("darude3.chrp");
    m_orchestra.addInstrument(m_winch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Hook BAR
   */
  
  public void raiseShaft() {

    m_piston.set(Value.kForward);
  }

  public void lowerShaft() {

    m_piston.set(Value.kReverse);
  }

  public boolean isShaftRaised() {

    return m_piston.get() == Value.kForward;
  }

  /**
   * Hook itself (which detaches)
   */

  public void extendShaft() {

    m_hooker.set(ControlMode.Position, ClimberConstants.kHookerDeployedPosition);
    m_hasHookDeployed = true;
  }

  public void retractShaft() {

    m_hooker.set(ControlMode.Position, 0);
  }

  public boolean isHookFullyExtended() {

    return ((Math.abs(ClimberConstants.kHookerDeployedPosition - m_hooker.getSelectedSensorPosition())) 
      / ClimberConstants.kHookerDeployedPosition) <= ClimberConstants.kHookerDeployedPercentTolerance;
  }

  public boolean isHookFullyRetracted() {

    return ((Math.abs(ClimberConstants.kHookerDeployedPosition - m_hooker.getSelectedSensorPosition())) 
      / ClimberConstants.kHookerDeployedPosition) >= (1.0 - ClimberConstants.kHookerDeployedPercentTolerance);
  }

  public void lift(double speed) {

    m_winch.set(ControlMode.Velocity, speed * ClimberConstants.kWinchOperationSpeed);
  }

  public void stopLifting() {

    m_winch.set(ControlMode.Velocity, 0.0);
  }

  private void resetEncoders() {

    m_hooker.getSensorCollection().setQuadraturePosition(0, Constants.kTimeout);
    m_winch.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeout);
  }

  public void playMusic() {

    m_orchestra.play();
  }

  public void stopMusic() {

    m_orchestra.stop();
  }

  public boolean hasHookDeployedOnce() {

    return m_hasHookDeployed;
  }
}
