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
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private final WPI_TalonSRX m_master = new WPI_TalonSRX(ShooterConstants.kTalonPortRight);
  private final WPI_TalonSRX m_slave = new WPI_TalonSRX(ShooterConstants.kTalonPortLeft); 

  /**
   * Creates a new SideShooter.
   */
  public Shooter() {

    TalonSRXConfiguration masterConfig = new TalonSRXConfiguration();

    // config feedback sensor
    masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

    // config gains
    masterConfig.slot0.kF = ShooterConstants.kGains.kF;
    masterConfig.slot0.kP = ShooterConstants.kGains.kP;
    masterConfig.slot0.kI = ShooterConstants.kGains.kI;
    masterConfig.slot0.kD = ShooterConstants.kGains.kD;

    masterConfig.nominalOutputReverse = 0.0;
    masterConfig.peakOutputReverse = 0.0;

    // apply configs
    m_master.configAllSettings(masterConfig);
    m_slave.configFactoryDefault();

    // phase sensor
    m_master.setSensorPhase(false);
    
    // slavery
    m_slave.follow(m_master);
    m_slave.setInverted(InvertType.OpposeMaster);

    final int amps = 80;
    m_master.configPeakCurrentLimit(amps);
    m_master.enableCurrentLimit(true);    
    m_slave.configPeakCurrentLimit(amps);
    m_slave.enableCurrentLimit(true);    
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Shooter RPM", getRPM());
    SmartDashboard.putNumber("Shooter percent error", getClosedLoopError());
  }

  public void setNative(int targetVelocity) {

    m_master.set(ControlMode.Velocity, targetVelocity);
  }

  public void setRPM(double rpm) {

    setNative(ShooterConstants.getVelocityNativeFromRPM(rpm));
  }

  public double getRPM() {
    
    return ShooterConstants.getRPMFromNativeVelocity(m_master.getSelectedSensorVelocity());
  }

  /**
   * NOT an absolute value. Negative indicated measurement is lower than setpoint!
   * @return
   */
  public double getClosedLoopError() {
    
    return ShooterConstants.getRPMFromNativeVelocity(m_master.getClosedLoopError());
  }

  public boolean isDippedPastShotThreshold() {

    // error percent is not absolute!!!
    // return getClosedLoopError() <= -ShooterConstants.kShootDipPercent;
    return false;
  }
}
