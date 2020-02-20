/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {

  private final WPI_TalonSRX m_conveyor = new WPI_TalonSRX(FeederConstants.kTalonPortConveyor);

  /**
   * Creates a new Feeder.
   */
  public Feeder() {

    m_conveyor.configFactoryDefault();

    m_conveyor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void feed() {
    
    m_conveyor.set(ControlMode.PercentOutput, 0.5);
  }

  public void dontFeed() {

    m_conveyor.set(ControlMode.PercentOutput, 0.0);
  }
}