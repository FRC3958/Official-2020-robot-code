/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private WPI_TalonSRX m_masterTalon = new WPI_TalonSRX(1);
  private WPI_TalonSRX m_slaveTalon = new WPI_TalonSRX(5);

  /**
   * Creates a new Shooter.
   */
  public Shooter() {

    m_masterTalon.setNeutralMode(NeutralMode.Brake);
    m_slaveTalon.setNeutralMode(NeutralMode.Brake);

    m_slaveTalon.follow(m_masterTalon);

    m_masterTalon.setInverted(InvertType.None);
    m_slaveTalon.setInverted(InvertType.OpposeMaster);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void fire(double speed) {
    m_masterTalon.set(speed);
  }
}
