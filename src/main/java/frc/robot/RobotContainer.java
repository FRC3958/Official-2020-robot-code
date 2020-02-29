/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.ControlConstants;
import frc.robot.commands.EatBalls;
import frc.robot.commands.LimelightModepresets;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.climbing.LiftBot;
import frc.robot.commands.climbing.PrepareClimb;
import frc.robot.commands.shooting.AlignToTarget;
import frc.robot.commands.shooting.FullShootRoutine;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HoodedShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.indexing.ConveyorBelt;
import frc.robot.subsystems.indexing.SideBelt;
import frc.robot.subsystems.indexing.StopWheel;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // OI
  private final XboxController m_driverController = new XboxController(ControlConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(ControlConstants.kOperatorControllerPort);

  // The robot's subsystems
  private final Drivetrain m_drive = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final HoodedShooter m_shooter = new HoodedShooter();
  private final Limelight m_limelight = new Limelight();
  private final Climber m_climber = new Climber();

  /**
   * Indexing subsystems
   */
  private final SideBelt m_indexer = new SideBelt();
  private final ConveyorBelt m_feeder = new ConveyorBelt();
  private final StopWheel m_stopWheel = new StopWheel();
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings(); 
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Drive with stick (note: it is automatically linearly limited)
    m_drive.setDefaultCommand(new ArcadeDrive(m_drive,
      () -> Util.deadband(m_driverController.getY(Hand.kRight), 0.1),
      () -> Util.deadband(m_driverController.getX(Hand.kLeft), 0.1))
    );

    // Toggle intake
    new JoystickButton(m_operatorController, ControlConstants.kKeybindToggleIntake)
      .toggleWhenPressed(new EatBalls(m_intake)
    );

    // Run entire shooting routine, maintaining alignment when held, and shoot balls
    // over and over again at the same time
    new JoystickButton(m_operatorController, ControlConstants.kKeybindShoot)
      .whenHeld(new AlignToTarget(m_limelight, m_drive, true))
      .whileHeld(new FullShootRoutine(m_shooter, m_indexer, m_feeder, m_stopWheel,
        () -> Util.calculateRPM(m_limelight.getApproximateDistance()))
    );

    // Prepare climber
    new JoystickButton(m_operatorController, ControlConstants.kKeybindPrepareClimb)
      .whenPressed(new PrepareClimb(m_climber)
    );

    // Climb
    new JoystickButton(m_operatorController, ControlConstants.kKeybindClimb)
      .whenPressed(new LiftBot(m_climber)
    );
    new JoystickButton(m_operatorController, ControlConstants.kKeybindMode)
      .toggleWhenPressed(new LimelightModepresets(m_limelight));

    /**
     * Shooter testing
     */

    // SmartDashboard.putNumber("native target", 1000);
    // SmartDashboard.putData("set native", 
    //   new InstantCommand(
    //     () -> m_shooter.setNative((int)SmartDashboard.getNumber("native target", 0)),
    //     m_shooter
    //   )
    // );

    // SmartDashboard.putNumber("rpm target", 500);
    // SmartDashboard.putData("set rpm", 
    //   new InstantCommand(
    //     () -> m_shooter.setRPM(SmartDashboard.getNumber("rpm target", 0)),
    //     m_shooter
    //   )
    // );

    // SmartDashboard.putData("zero shooter", new InstantCommand(() -> m_shooter.setNative(0), m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  } 
}
