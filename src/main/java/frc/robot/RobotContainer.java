/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.commands.EatBalls;
import frc.robot.commands.UnEatBalls;
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
import frc.robot.subsystems.indexing.Gateway;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
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
  private final Gateway m_gateway = new Gateway();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Drive with stick (note: it is automatically linearly limited)
    m_drive.setDefaultCommand(new ArcadeDrive(m_drive, () -> Util.deadband(m_driverController.getY(Hand.kRight), 0.1),
        () -> Util.deadband(m_driverController.getX(Hand.kLeft), 0.1)));

    // Toggle intake
    new JoystickButton(m_operatorController, ControlConstants.kKeybindToggleIntake)
        .toggleWhenPressed(new EatBalls(m_intake));

    new JoystickButton(m_operatorController, ControlConstants.kKeybindUnintake)
    .toggleWhenPressed(new UnEatBalls(m_intake)
      );
    // Run entire shooting routine, maintaining alignment when held, and shoot balls
    // over and over again at the same time
    new JoystickButton(m_operatorController, ControlConstants.kKeybindShoot)
      .whenHeld(new AlignToTarget(m_limelight, m_drive, true))
      .whileHeld(new FullShootRoutine(m_shooter, m_indexer, m_feeder, m_gateway,
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
  

    /**
     * Shooter testing
     */

    SmartDashboard.putNumber("native target", 1000);
    SmartDashboard.putData("set native", 
      new InstantCommand(
        () -> m_shooter.setNative((int)SmartDashboard.getNumber("native target", 0)),
        m_shooter
      )
    );

    SmartDashboard.putNumber("rpm target", 500);
    SmartDashboard.putData("set rpm", 
      new InstantCommand(
        () -> m_shooter.setRPM(SmartDashboard.getNumber("rpm target", 0)),
        m_shooter
      )
    );

    SmartDashboard.putData("zero shooter", new InstantCommand(() -> m_shooter.setNative(0), m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    var voltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        DriveConstants.ksVolts, 
        DriveConstants.kvVoltSecondsPerMeter, 
        DriveConstants.kaVoltSecondsSquaredPerMeter
      ),
      DriveConstants.kKinematics,
      10 // not 12, account for voltage sag
    );

    var config = new TrajectoryConfig(
      DriveConstants.kMaxVelocityMPS * .6, // 60% of max physical speed
      DriveConstants.kMaxVelocityMPS / 3.0 // 3 seconds to reach max speed, deemed safe for auton
    ).setKinematics(DriveConstants.kKinematics)
    .addConstraint(voltageConstraint);


    var trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        // internal waypoints
      ),
      new Pose2d(3, 0, new Rotation2d(0)),
      config
    );

    var ramsete = new RamseteCommand(
      trajectory, 
      m_drive::getPose, 
      new RamseteController(AutoConstants.kRameseteB, AutoConstants.kRameseteZeta),
      new SimpleMotorFeedforward(
        DriveConstants.ksVolts, 
        DriveConstants.kvVoltSecondsPerMeter, 
        DriveConstants.kaVoltSecondsSquaredPerMeter
      ), 
      DriveConstants.kKinematics, 
      m_drive::getWheelSpeeds, 
      new PIDController(DriveConstants.kPDriveVel, 0.0, 0.0),
      new PIDController(DriveConstants.kPDriveVel, 0.0, 0.0),
      m_drive::tankDriveVolts, 
      m_drive
    );

    return ramsete.andThen(() -> m_drive.tankDriveVolts(0.0, 0.0));

    // return null;
  } 
}
