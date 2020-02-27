/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.HoodedShooter;



public class SpinUpToSpeed extends CommandBase {
  
  private final HoodedShooter m_shooter;
  private final DoubleSupplier m_rpm;
  

  /**
   * Creates a new ArmShooter.
   */
  public SpinUpToSpeed(HoodedShooter shooter, DoubleSupplier rpm) {
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
    
    m_shooter.setRPM(m_rpm.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_shooter.getClosedLoopErrorPercent()) <= ShooterConstants.kAcceptablePercentError;
  }
}
