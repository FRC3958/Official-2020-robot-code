/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArcadeDrive extends CommandBase {

  private final Drivetrain m_drive;
  private final DoubleSupplier m_forward, m_turn;

  private final SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_turnLimiter = new SlewRateLimiter(10);

  private final PIDController m_straightPid = new PIDController(0.5, 0, 0);
  private boolean m_justRanPid = false;
  private double m_angleSetpoint = 0.0;

  /**
   * Creates a new StickDrive.
   */
  public ArcadeDrive(Drivetrain drive, DoubleSupplier forward, DoubleSupplier turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    m_drive = drive;
    m_forward = forward;
    m_turn = turn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double turn = m_turnLimiter.calculate(m_turn.getAsDouble());

    // if(turn == 0) {

    //   SmartDashboard.putBoolean("Driving straight", true);

    //   // set new heading for driving straight if just starting, otherwise maintain heading
    //   if(!m_justRanPid) {
    //     m_angleSetpoint = m_drive.getHeading();
    //   }

    //   turn = m_straightPid.calculate(m_drive.getHeading(), m_angleSetpoint);

    //   m_justRanPid = true;
    // } else {

      SmartDashboard.putBoolean("Driving straight", false);

      m_justRanPid = false;
    // }

    m_drive.arcadeDrive(
      m_forwardLimiter.calculate(m_forward.getAsDouble()), 
      turn
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drive.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
