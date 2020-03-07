/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;;

public class SeekTarget extends CommandBase {

  private final Drivetrain m_drive;
  private final Limelight m_limelight;

  /**
   * Creates a new SeekTarget.
   */
  public SeekTarget(Drivetrain drive, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    m_drive = drive;
    m_limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drive.arcadeDrive(0.0, .4);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    m_drive.arcadeDrive(0.0, .4);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return m_limelight.isValidTargetPresent();
  }
}
