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
import frc.robot.Constants.IndexerConstants;

public class SideBelt extends SubsystemBase {

  private final WPI_TalonSRX m_sideways = new WPI_TalonSRX(IndexerConstants.kTalonPortSideways);

  /**
   * Creates a new Indexer.
   */
  public SideBelt() {

    m_sideways.configFactoryDefault();

    m_sideways.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinSideways() {
    m_sideways.set(ControlMode.PercentOutput, IndexerConstants.kRunningPercentOutput);
  }

  public void stopSpinningSideways() {
    m_sideways.set(ControlMode.PercentOutput, 0.0);
  }
}
