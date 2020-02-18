/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HoodedShooter;

public class SpinUntilShot extends CommandBase {

  private final HoodedShooter m_shooter;
  private final double m_rpm;

  private final Timer m_timer = new Timer();

  /**
   * Creates a new TriggerShooter.
   */
  public SpinUntilShot(HoodedShooter shooter, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    m_shooter = shooter;
    m_rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.setRPM(m_rpm);    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_shooter.isDippedPastThreshold()) {
      m_timer.start();
    }
    else {
      m_timer.reset();
    }

    return m_timer.get() > ShooterConstants.kTimeToShoot;
  }
}
