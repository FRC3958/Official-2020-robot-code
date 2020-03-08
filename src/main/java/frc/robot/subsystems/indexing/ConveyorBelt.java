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
import frc.robot.constants.ConveyorBeltConstants;

public class ConveyorBelt extends SubsystemBase {

  private final WPI_TalonSRX m_belt = new WPI_TalonSRX(ConveyorBeltConstants.kTalonPort);

  /**
   * Feeds into the shooter wheel.
   */
  public ConveyorBelt() {

    m_belt.configFactoryDefault();

    m_belt.configPeakCurrentLimit(20);    
    m_belt.enableCurrentLimit(true);    

    m_belt.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void spin() {
    
    m_belt.set(ControlMode.PercentOutput, ConveyorBeltConstants.kRunningPercentOutput);
  }

  public void stop() {

    m_belt.set(ControlMode.PercentOutput, 0.0);
  }
}
