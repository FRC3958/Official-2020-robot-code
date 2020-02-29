/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.Limelight.LedMode;
//import frc.robot.subsystems.Limelight.CamMode;



public class LimelightModepresets extends CommandBase {
  /**
   * Creates a new LimelightModepresets.
   */
  private Limelight m_limelight;
 
  public LimelightModepresets(Limelight limelight ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_limelight.setLedMode(3);
    m_limelight.setCamMode(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.setLedMode(1);
    m_limelight.setCamMode(1);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
