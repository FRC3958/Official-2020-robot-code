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
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {

  private final WPI_TalonSRX m_master = new WPI_TalonSRX(kTalonPortRight);
  private final WPI_TalonSRX m_slave = new WPI_TalonSRX(kTalonPortLeft); 

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private int m_lastProximity = 0;
  private int m_ballsShot = 0;

  /**
   * Creates a new SideShooter.
   */
  public Shooter() {

    TalonSRXConfiguration masterConfig = new TalonSRXConfiguration();

    // config feedback sensor
    masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

    // config gains
    masterConfig.slot0.kF = kGains.kF;
    masterConfig.slot0.kP = kGains.kP;
    masterConfig.slot0.kI = kGains.kI;
    masterConfig.slot0.kD = kGains.kD;

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
    
    updateBallCount();

    SmartDashboard.putNumber("Shooter RPM", getRPM());
    SmartDashboard.putNumber("Shooter error", getClosedLoopErrorRPM());
    SmartDashboard.putNumber("Balls shot", getBallsShot());
    SmartDashboard.putNumber("Proximity reading", m_colorSensor.getProximity());
  }

  private void updateBallCount() {

    // Proximity measurement value, ranging from 0 to 2047
    // This value is largest when an object is close to the sensor and smallest when far away.

    int proximity = m_colorSensor.getProximity();

    // TODO: find appropriate values
   if(m_lastProximity < 500 && proximity > 1700) {
      ++m_ballsShot;
    }
    else if (getRPM() == 0){
      m_ballsShot = 0;
    }

    m_lastProximity = proximity;
  }

  public void setNative(int targetVelocity) {

    m_master.set(ControlMode.Velocity, targetVelocity);
  }

  public void setRPM(double rpm) {

    setNative(getVelocityNativeFromRPM(rpm));
  }

  public double getRPM() {

    return getRPMFromNativeVelocity(m_master.getSelectedSensorVelocity());
  }

  public double getClosedLoopErrorRPM() {
    
    return getRPMFromNativeVelocity(m_master.getClosedLoopError());
  }

  public int getBallsShot() {

    return m_ballsShot;
  }
}
