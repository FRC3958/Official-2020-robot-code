/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousRoutine;
import frc.robot.commands.shooting.AlignToTarget;
import frc.robot.commands.shooting.FullShootRoutine;
import frc.robot.constants.Controls;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.indexing.Conveyor;
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
  private final XboxController m_driverController = new XboxController(Controls.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(Controls.kOperatorControllerPort);

  // The robot's subsystems
  private final Drivetrain m_drive = new Drivetrain();
  
  private final Intake m_intake = new Intake();
  
  private final Shooter m_shooter = new Shooter();
  
  private final Limelight m_limelight = new Limelight();
  
  private final Climber m_climber = new Climber();

  private final SideBelt m_sideBelt = new SideBelt();
  private final Conveyor m_conveyor = new Conveyor();
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

    // SmartDashboard.putData(new PowerDistributionPanel());

    // /**
    //  * Intake testing
    //  */

    // SmartDashboard.putData("Drop intake", new DropIntake(m_intake));
    // SmartDashboard.putData("Lift intake", new LiftIntake(m_intake));

    // SmartDashboard.putData("Eat balls", new EatBalls(m_intake));
    // SmartDashboard.putData("Eject balls", new EjectBalls(m_intake));

    // /**
    //  * Indexing testing
    //  */
    
    // SmartDashboard.putData("Feed to gateway (from sidebelt)", 
    //   new FeedToGateway(m_sideBelt));
    // SmartDashboard.putData("Load to conveyor (from gateway)", 
    //   new LoadToConveyor(m_gateway));
    // SmartDashboard.putData("Feed to shooter (from conveyor)",
    //   new FeedToShooter(m_conveyor));

    // /*
    //  * Shooter testing
    //  */

    // SmartDashboard.putNumber("Test RPM setpoint", 0);
    // SmartDashboard.putData("Set RPM setpoint", 
    //   new InstantCommand(
    //     () -> m_shooter.setRPM(SmartDashboard.getNumber("Test RPM setpoint", 0)),
    //     m_shooter
    //   )
    // );

    // SmartDashboard.putData("Zero shooter setpoint", 
    //   new InstantCommand(() -> m_shooter.setNative(0), m_shooter)
    // );

    // /**
    //  * Climber testing
    //  */

    // SmartDashboard.putData("Raise shaft", new RaiseShaft(m_climber));
    // SmartDashboard.putData("Lower shaft", new LowerShaft(m_climber));

    // SmartDashboard.putData("Extend hook shaft", new ExtendShaft(m_climber));
    // SmartDashboard.putData("Retract hook shaft", new RetractShaft(m_climber));

    // SmartDashboard.putData("Lift bot (hook must be deployed)", new LiftBot(m_climber, () -> 0.3));

    // /**
    //  * Limelight
    //  */

    // SmartDashboard.putData("Turn lights off", new InstantCommand(() -> m_limelight.setLedMode(LedMode.kForceOff), m_limelight));
    // SmartDashboard.putData("Turn lights on", new InstantCommand(() -> m_limelight.setLedMode(LedMode.kForceOn), m_limelight));   

    // /**
    //  * Music
    //  */

    // SmartDashboard.putData("Play music", new InstantCommand(() -> m_climber.playMusic()));
    // SmartDashboard.putData("Pause music", new InstantCommand(() -> m_climber.stopMusic()));

    // SmartDashboard.putData("Test fire", new FullShootRoutine(m_shooter, m_conveyor, m_gateway,
    //   () -> Util.calculateRPM(m_limelight.getApproximateDistanceMeters()) + 100
    // ));

    // SmartDashboard.putData("Align to target", new AlignToTarget(m_limelight, m_drive, true, () -> 0.0));
  }

  /**
   * Configures button bindings (duh)
   */
  private void configureButtonBindings() {

    // config controls
    configureDriverControls(m_driverController, m_operatorController);
    configureOperatorControls(m_operatorController, m_driverController);
  }

  /**
   * Configures given controller for driver controls
   * @param controller
   * @param operatorController
   */
  private void configureDriverControls(XboxController controller, XboxController operatorController) {
    
    // Drive with stick (note: it is automatically linearly limited)
    m_drive.setDefaultCommand(new ArcadeDrive(m_drive, 
      () -> Util.deadband(-controller.getRawAxis(Controls.Driver.kForwardDrive), 0.1),
      () -> Util.deadband(controller.getRawAxis(Controls.Driver.kTurnDrive), 0.1),
      () -> controller.getRawButton(Controls.Driver.kHalfSpeed))
    );

    // Auto align to target
    new JoystickButton(controller, Controls.Driver.kAutoAlign)
      .whenHeld(new AlignToTarget(m_limelight, m_drive, true,
        () -> Util.deadband(controller.getRawAxis(Controls.Driver.kForwardDrive), 0.1))
    );
  }

  /**
   * Configure all operator controls on the given controller
   * @param controller
   * @param driverController
   */
  private void configureOperatorControls(XboxController controller, XboxController driverController) {

    /**
     * Intake & indexing
     */

    // Intake control
    new Button(() -> Math.abs(controller.getRawAxis(Controls.Operator.kEat)) >= 0.2)
      .whenPressed(() -> m_intake.spin(controller.getRawAxis(Controls.Operator.kEat)), m_intake)
      .whenReleased(() -> m_intake.stopSpinning(), m_intake);

    // Toggle intake piston
    new JoystickButton(controller, Controls.Operator.kLiftLowerIntake)
      .whenPressed(() -> { m_intake.stopSpinning(); m_intake.toggleBar(); }, m_intake
    );

    // Sidebelt control (indexer)
    new Button(() -> Math.abs(controller.getRawAxis(Controls.Operator.kSpinSidebelt)) >= 0.2)
      .whenPressed(() -> m_sideBelt.spin(controller.getRawAxis(Controls.Operator.kSpinSidebelt) * .5), m_sideBelt)
      .whenReleased(() -> m_sideBelt.stop()
    );

    /**
     * Shooting
     */

    // Run entire shooting routine using xuru (PRAISE BE)
    new Button(() -> controller.getRawAxis(Controls.Operator.kShoot) >= 0.5)
      .whenHeld(new FullShootRoutine(m_shooter, m_conveyor, m_gateway,
        () -> Util.calculateRPM(m_limelight.getApproximateDistanceMeters())))
      .whileHeld(() -> m_limelight.resetLedTimer()
    );

    /**
     * Climbing
     */

    // Toggle raising climber shaft
    new JoystickButton(controller, Controls.Operator.kToggleClimberPiston)
      .whenPressed(() -> m_climber.toggleShaft(), m_climber
    );

    // Extend shaft
    new JoystickButton(controller, Controls.Operator.kExtendShaft)
      .whenPressed(() -> m_climber.extendShaft(), m_climber)
      .whenReleased(() -> m_climber.stopExtending(), m_climber
    );

    // Retract shaft
    new JoystickButton(controller, Controls.Operator.kRetractShaft)
      .whenPressed(() -> m_climber.retractShaft(), m_climber)
      .whenReleased(() -> m_climber.stopExtending(), m_climber
    );

    // Lift (with winch)
    new Button(() -> Math.abs(controller.getRawAxis(Controls.Operator.kLift)) >= 0.4)
      .whenPressed(() -> m_climber.lift(-controller.getRawAxis(Controls.Operator.kLift)), m_climber)
      .whenReleased(() -> m_climber.stopLifting(), m_climber
    );

    /**
     * Misc
     */
    
    // Switch camera mode on limelight (for use as a normal camera)
    // new JoystickButton(controller, Controls.Operator.kLimelightModeSwitch)
    //   .whenPressed(() -> { 

    //     int camMode, ledMode;

    //     if(m_limelight.getCamMode() == CamMode.kVisionProcessor) {
    //       camMode = CamMode.kDriver;
    //       ledMode = LedMode.kForceOff; 
    //     } else {
    //       camMode = CamMode.kVisionProcessor;
    //       ledMode = LedMode.kPipeline;
    //     }
    //     m_limelight.setCamMode(camMode);
    //     m_limelight.setLedMode(ledMode);
    //   }, m_limelight
    // );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return new AutonomousRoutine(m_drive, m_limelight, m_shooter, m_conveyor, m_gateway, m_intake)
      .andThen(() -> m_drive.arcadeDrive(0, 0), m_drive); // make sure we are stopped at end of auton
  }
}
