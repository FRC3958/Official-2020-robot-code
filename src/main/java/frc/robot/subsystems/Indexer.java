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
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  private final WPI_TalonSRX m_sideways = new WPI_TalonSRX(IndexerConstants.kTalonPortSideways);
  private final WPI_TalonSRX m_stopWheel = new WPI_TalonSRX(IndexerConstants.kTalonPortTopWheel);

  /**
   * Creates a new Indexer.
   */
  public Indexer() {

    m_sideways.configFactoryDefault();
    m_stopWheel.configFactoryDefault();

    m_sideways.setNeutralMode(NeutralMode.Coast);
    m_stopWheel.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinSideways(double speed) {
    m_sideways.set(ControlMode.PercentOutput,speed);
  }

  public void stopSpinningSideways() {
    m_sideways.set(ControlMode.PercentOutput, 0.0);
  }

  public void disengangeStopWheel() {
    m_stopWheel.set(ControlMode.PercentOutput, 0.3);
  }

  public void engageStopWheel() {
    m_stopWheel.set(ControlMode.PercentOutput, 0.0);
  }
}
