/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HoodedShooter;

public class SpinUpToSpeed extends CommandBase {

  private final HoodedShooter m_shooter;
  private final Double m_rpm;

  private final Timer m_timer = new Timer();

  /**
   * Creates a new TriggerShooter.
   */
  public SpinUpToSpeed(HoodedShooter shooter, double testrpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    m_shooter = shooter;
    m_rpm = testrpm;

    m_timer.reset();
    m_timer.stop();
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
    
    // TODO: figure out a way to account for a changing target RPM...
    //if(m_shooter.isDippedPastShotThreshold()) {
      //m_timer.start();
    //}

    return m_timer.get() > ShooterConstants.kTimeToShoot;
  }
}