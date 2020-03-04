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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.EatBalls;
import frc.robot.commands.SwitchToDriverMode;
import frc.robot.commands.EjectBalls;
import frc.robot.commands.climbing.ExtendShaft;
import frc.robot.commands.climbing.LiftBot;
import frc.robot.commands.climbing.LowerShaft;
import frc.robot.commands.climbing.RaiseShaft;
import frc.robot.commands.climbing.RetractShaft;
import frc.robot.commands.shooting.AlignToTarget;
import frc.robot.commands.shooting.FullShootRoutine;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.indexing.ConveyorBelt;
import frc.robot.subsystems.indexing.Gateway;
import frc.robot.subsystems.indexing.SideBelt;

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
  
  private final Shooter m_shooter = new Shooter();
  
  private final Limelight m_limelight = new Limelight();
  
  private final Climber m_climber = new Climber();

  private final SideBelt m_sideBelt = new SideBelt();
  private final ConveyorBelt m_conveyor = new ConveyorBelt();
  private final Gateway m_gateway = new Gateway();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    setupSmartDashboard();
    configureButtonBindings();
  }

  /**
   * Puts all information to SmartDashboard. Putting anything general
   */
  private void setupSmartDashboard() {

    /*
     * Shooter testing
     */

    SmartDashboard.putNumber("RPM setpoint", 0);
    SmartDashboard.putData("Set RPM setpoint", 
      new InstantCommand(
        () -> m_shooter.setRPM(SmartDashboard.getNumber("RPM setpoint", 0)),
        m_shooter
      )
    );

    SmartDashboard.putData("Zero shooter", 
      new InstantCommand(() -> m_shooter.setNative(0), m_shooter)
    );

    /**
     * Climber testing
     */

    SmartDashboard.putData("Extend hook shaft", new ExtendShaft(m_climber));
    SmartDashboard.putData("Retract hook shaft", new RetractShaft(m_climber));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // config controls
    configureDriverControls(m_driverController);
    configureOperatorControls(m_operatorController);
  }

  /**
   * Configure all driver controls on the given controller
   * @param controller
   */
  private void configureDriverControls(XboxController controller) {
    
    // Drive with stick (note: it is automatically linearly limited)
    m_drive.setDefaultCommand(new ArcadeDrive(m_drive, 
      () -> Util.deadband(m_driverController.getY(Hand.kRight), 0.1),
      () -> Util.deadband(m_driverController.getX(Hand.kLeft), 0.1))
    );
  }

  /**
   * Configure all operator controls on the given controller
   * @param controller
   */
  private void configureOperatorControls(XboxController controller) {

    // Toggle intake
    new JoystickButton(m_operatorController, ControlConstants.Operator.kToggleIntake)
      .toggleWhenPressed(new EatBalls(m_intake)
    );

    // Clear intake
    new JoystickButton(m_operatorController, ControlConstants.Operator.kEjectBalls)
      .toggleWhenPressed(new EjectBalls(m_intake)
    );

    // Run entire shooting routine, maintaining alignment when held, and shoot balls
    // over and over again at the same time
    new JoystickButton(m_operatorController, ControlConstants.Operator.kShoot)
      .whenHeld(new AlignToTarget(m_limelight, m_drive, true))
      .whileHeld(new FullShootRoutine(m_shooter, m_sideBelt, m_conveyor, m_gateway,
        () -> Util.calculateRPM(m_limelight.getApproximateDistanceMeters()))
    );

    // Lift shaft and put hook above bar
    new JoystickButton(m_operatorController, ControlConstants.Operator.kPrepareClimb)
      .whenHeld(
        new RaiseShaft(m_climber)               // first raise shaft (with solenoid)
        .andThen(new ExtendShaft(m_climber))    // then extend shaft (like a toy lightsaber)
      )
      .whenReleased(new RetractShaft(m_climber) // retract hook when released
    );

    // Climb
    new JoystickButton(m_operatorController, ControlConstants.Operator.kClimb)
      .whenPressed(new LowerShaft(m_climber)) // first lower shaft
      .whenHeld(new LiftBot(m_climber)        // lift bot while pressing
    );    

    // Switch driver mode limelight
    new JoystickButton(m_operatorController, ControlConstants.Operator.kLimelightModeSwitch)
      .toggleWhenPressed(new SwitchToDriverMode(m_limelight)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // var voltageConstraint = new DifferentialDriveVoltageConstraint(
    //   new SimpleMotorFeedforward(
    //     DriveConstants.ksVolts, 
    //     DriveConstants.kvVoltSecondsPerMeter, 
    //     DriveConstants.kaVoltSecondsSquaredPerMeter
    //   ),
    //   DriveConstants.kKinematics,
    //   10 // not 12, account for voltage sag
    // );

    // var config = new TrajectoryConfig(
    //   DriveConstants.kMaxVelocityMPS * .6, // 60% of max physical speed
    //   DriveConstants.kMaxVelocityMPS / 3.0 // 3 seconds to reach max speed, deemed safe for auton
    // ).setKinematics(DriveConstants.kKinematics)
    // .addConstraint(voltageConstraint);


    // var trajectory = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, new Rotation2d(0)),
    //   List.of(
    //     // internal waypoints
    //   ),
    //   new Pose2d(3, 0, new Rotation2d(0)),
    //   config
    // );

    // var ramsete = new RamseteCommand(
    //   trajectory, 
    //   m_drive::getPose, 
    //   new RamseteController(AutoConstants.kRameseteB, AutoConstants.kRameseteZeta),
    //   new SimpleMotorFeedforward(
    //     DriveConstants.ksVolts, 
    //     DriveConstants.kvVoltSecondsPerMeter, 
    //     DriveConstants.kaVoltSecondsSquaredPerMeter
    //   ), 
    //   DriveConstants.kKinematics, 
    //   m_drive::getWheelSpeeds, 
    //   new PIDController(DriveConstants.kPDriveVel, 0.0, 0.0),
    //   new PIDController(DriveConstants.kPDriveVel, 0.0, 0.0),
    //   m_drive::tankDriveVolts, 
    //   m_drive
    // );

    // return ramsete.andThen(() -> m_drive.tankDriveVolts(0.0, 0.0));

    return null;
  } 
}
