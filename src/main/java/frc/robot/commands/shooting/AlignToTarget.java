/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.CamMode;
import frc.robot.subsystems.Limelight.LedMode;

public class AlignToTarget extends PIDCommand {

  private final boolean m_forever;
  private final Limelight m_limelight;

  /**
   * Creates a new AlignToTarget.
   */
  public AlignToTarget(Limelight limelight, Drivetrain drivetrain, boolean forever, DoubleSupplier forward) {    
    
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
          drivetrain.arcadeDrive(forward.getAsDouble(), -output);
        });
        
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, drivetrain);

    getController().enableContinuousInput(-180, 180);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(
      VisionConstants.kAlignToTargetTolerancePosition,
      VisionConstants.kAlignToTargetToleranceVelocity
    );

    m_forever = forever;
    m_limelight = limelight;

    m_limelight.setLedMode(LedMode.kForceOn);
    m_limelight.setCamMode(CamMode.kVisionProcessor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_forever ? false : getController().atSetpoint();
  }
}
