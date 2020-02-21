/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.StopWheel;

public class LoadToConveyor extends CommandBase {
  
  private final StopWheel m_stopWheel;
  
  /**
   * Loads ball into the conveyor
   */
  public LoadToConveyor(StopWheel stopWheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(stopWheel);

    m_stopWheel = stopWheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_stopWheel.disengangeStopWheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_stopWheel.engageStopWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}