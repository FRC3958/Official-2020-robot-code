/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.indexing.SideBelt;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClearIndexer extends CommandBase {

  private final SideBelt m_sidebelt;
  private final DoubleSupplier m_speed;

  /**
   * Creates a new ClearIndexer.
   */
  public ClearIndexer(SideBelt sidebelt, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sidebelt);

    m_sidebelt = sidebelt;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_sidebelt.spin(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_sidebelt.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
