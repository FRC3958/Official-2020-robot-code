/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.HoodedShooter;

public class FeedBallUntilShot extends CommandBase {

  private final Feeder m_feeder;
  private final HoodedShooter m_shooter;

  /**
   * Creates a new FeedBall.
   */
  public FeedBallUntilShot(Feeder feeder, HoodedShooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder); // we do not require exclusive control of shooter!!!

    m_feeder = feeder;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_feeder.feed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_feeder.dontFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.isDippedPastShotThreshold();
  }
}
