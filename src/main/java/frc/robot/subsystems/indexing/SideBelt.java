/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.indexing;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.SideBeltConstants.*;

public class SideBelt extends SubsystemBase {

  private final WPI_TalonSRX m_sideways = new WPI_TalonSRX(kTalonPort);

  /**
   * Clears jams and places balls into the gateway
   */
  public SideBelt() {

    m_sideways.configFactoryDefault();
    
    m_sideways.configPeakCurrentLimit(15);
    m_sideways.enableCurrentLimit(true);    

    m_sideways.setInverted(InvertType.None);
    m_sideways.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin() {
    m_sideways.set(ControlMode.PercentOutput, kRunningPercentOutput);
  }

  public void spin(double speed) {
    m_sideways.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    m_sideways.set(ControlMode.PercentOutput, 0.0);
  }
}
