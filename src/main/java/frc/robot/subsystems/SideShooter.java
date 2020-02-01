/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class SideShooter extends PIDSubsystem {

  private final WPI_TalonSRX m_leftTalon = new WPI_TalonSRX(7);
  private final WPI_TalonSRX m_rightTalon = new WPI_TalonSRX(0); 

  /**
   * Creates a new SideShooter.
   */
  public SideShooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(.5f, 0, 0)
    );

    SmartDashboard.putData("sideShooterPid", getController());

    m_leftTalon.configFactoryDefault();
    m_rightTalon.configFactoryDefault();

    m_leftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_rightTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    m_leftTalon.setSensorPhase(true);

    getController().setTolerance(.01f);
    getController().setSetpoint(1.f);
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();

    SmartDashboard.putNumber("imbalance", getOutputImbalance());
  }

  /**
   * Positive imbalance indicates right is overpowered, and vice versa
   */
  private double getOutputImbalance() {
    return (double)m_rightTalon.getSelectedSensorVelocity() / (double)m_leftTalon.getSelectedSensorVelocity();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_leftTalon.set(output);
    m_rightTalon.set(-output);

    System.out.println(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getOutputImbalance();
  }

  public void setSpeed(double speed) {
    m_leftTalon.set(speed);
    m_rightTalon.set(speed);
  }
}
