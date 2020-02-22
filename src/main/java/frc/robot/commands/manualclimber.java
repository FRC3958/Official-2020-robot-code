/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class manualclimber extends CommandBase {
  /**
   * Creates a new manualclimber.
   */
  private Climber m_climber;
  double speedo;
  double speedt;
  public manualclimber(Climber c, double talonspeed,double falconspeed) {
    // Use addRequirements() here to declare subsystem dependencies.
   talonspeed = speedo;
   falconspeed = speedt;
    m_climber = c;
    addRequirements(c);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.manualcontrol(speedo, speedt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.manualcontrol(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
