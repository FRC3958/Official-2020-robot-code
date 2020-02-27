/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignToTarget extends PIDCommand {

  private final boolean m_forever;

  /**
   * Creates a new AlignToTarget.
   */
  public AlignToTarget(Limelight limelight, Drivetrain drivetrain, boolean forever) {    
    
    super(
        // The controller that the command will use
        
        new PIDController(
          VisionConstants.kAlignToTargetGains.kP, 
          VisionConstants.kAlignToTargetGains.kI, 
          VisionConstants.kAlignToTargetGains.kD
        ),
        // This should return the measurement
        () -> limelight.getBestAngleOffsetX(),
        // This should return the setpoint (can also be a constant)
        () -> 0.0,
        // This uses the output
        output -> {
          // Use the output here
          output = MathUtil.clamp(output, -1.0, 1.0);
          drivetrain.arcadeDrive(0.0, output);
        });
        
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, drivetrain);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(
      VisionConstants.kAlignToTargetTolerancePosition,
      VisionConstants.kAlignToTargetToleranceVelocity
    );

    m_forever = forever;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_forever ? false : getController().atSetpoint();
  }
}
