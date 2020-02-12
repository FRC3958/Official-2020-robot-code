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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class HoodedShooter extends SubsystemBase {

  private final WPI_TalonSRX m_master = new WPI_TalonSRX(7);
  private final WPI_TalonSRX m_slave = new WPI_TalonSRX(0); 

  /**
   * Creates a new SideShooter.
   */
  public HoodedShooter() {

    m_master.configFactoryDefault();
    m_slave.configFactoryDefault();

    // config encoder
    m_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    // phase sensor
    m_master.setSensorPhase(true);

    // config nominal and peak outputs
    m_master.configNominalOutputForward(0);
    m_master.configNominalOutputReverse(0);
    m_master.configPeakOutputForward(1);
    m_master.configPeakOutputReverse(-1);

    // config gains
    // running the loop on the talon
    m_master.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.kF);
    m_master.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kP);
    m_master.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kI);
    m_master.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kD);
    
    // slavery
    m_slave.follow(m_master);
    m_slave.setInverted(InvertType.OpposeMaster);
  }

  public void set(double setpointRpm) {

    var targetVelocity = setpointRpm * 4096 // to native units per minute
      / 60 / 10;
    // 1min / 60 = 1s, 1s / 10 = 100ms, native units per 100ms
      
    m_master.set(ControlMode.Velocity, targetVelocity);
  }
}
