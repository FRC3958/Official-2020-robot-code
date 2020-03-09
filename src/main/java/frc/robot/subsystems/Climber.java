/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {

  private final WPI_TalonSRX m_hooker = new WPI_TalonSRX(ClimberConstants.kTalonPortHooker);
  private final WPI_TalonFX m_winch = new WPI_TalonFX(ClimberConstants.kTalonPortLifter);

  private final DoubleSolenoid m_piston = new DoubleSolenoid(ClimberConstants.kPCMPistonForward, ClimberConstants.kPCMPistonReverse);

  /**
   * Creates a new Climber.
   */
  public Climber() {

    m_hooker.configFactoryDefault();
    m_winch.configFactoryDefault();    
    
    m_winch.setNeutralMode(NeutralMode.Brake);
    m_winch.setInverted(InvertType.None);
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

  public void toggleShaft() {

    if(isShaftRaised()) {
      lowerShaft();
    } else {
      raiseShaft();
    }
  }

  /**
   * Hook itself (which detaches)
   */

  public void stopExtending() {

    m_hooker.set(ControlMode.PercentOutput, 0.0);
  }

  public void extendShaft() {

    m_hooker.set(ControlMode.PercentOutput, 0.35);
  }

  public void retractShaft() {

    m_hooker.set(ControlMode.PercentOutput, -0.4);
  }

  public void lift(double speed) {

    m_winch.set(ControlMode.PercentOutput, speed * ClimberConstants.kWinchOperationSpeed);
  }

  public void stopLifting() {

    m_winch.set(ControlMode.Velocity, 0.0);
  }
}
