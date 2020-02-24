/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.indexing;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StopWheelConstants;

public class StopWheel extends SubsystemBase {

  private final WPI_TalonSRX m_wheel = new WPI_TalonSRX(StopWheelConstants.kTalonPort);

  /**
   * Creates a new StopWheel.
   */
  public StopWheel() {

    m_wheel.configFactoryDefault();
    m_wheel.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void disengangeStopWheel() {
    m_wheel.set(ControlMode.PercentOutput, StopWheelConstants.kRunningPercentOutput);
  }

  public void engageStopWheel() {
    m_wheel.set(ControlMode.PercentOutput, 0.0);
  }
}
