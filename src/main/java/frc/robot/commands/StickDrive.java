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

public class StickDrive extends CommandBase {

  private final Drivetrain m_drive;
  private final DoubleSupplier m_forward, m_turn;

  private final SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_turnLimiter = new SlewRateLimiter(2);

  /**
   * Creates a new StickDrive.
   */
  public StickDrive(Drivetrain drive, DoubleSupplier forward, DoubleSupplier turn) {
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

    m_drive.arcadeDrive(
      m_forwardLimiter.calculate(m_forward.getAsDouble()), 
      m_turnLimiter.calculate(m_turn.getAsDouble())
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
