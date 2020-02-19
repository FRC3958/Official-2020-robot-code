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
import frc.robot.Constants.ControlConstants;
import frc.robot.commands.EatBalls;
import frc.robot.commands.StickDrive;
import frc.robot.commands.shooting.AlignToTarget;
import frc.robot.commands.shooting.ShootAtRPM;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HoodedShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

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
  private final HoodedShooter m_shooter = new HoodedShooter();
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  private final Limelight m_limelight = new Limelight();

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

    m_drive.setDefaultCommand(new StickDrive(m_drive,
      () -> Util.deadband(m_driverController.getY(Hand.kRight), 0.1),
      () -> Util.deadband(m_driverController.getX(Hand.kLeft), 0.1))
    );

    new JoystickButton(m_operatorController, ControlConstants.kKeybindToggleIntake)
      .toggleWhenPressed(new EatBalls(m_intake)
    );

    new JoystickButton(m_operatorController, ControlConstants.kKeybindShoot)
      .whenHeld(new AlignToTarget(m_limelight, m_drive, true))
      .whileHeld(new ShootAtRPM(m_shooter, m_indexer, 
        () -> Util.calculateRPM(m_limelight.getApproximateDistance()))
    );
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
