/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.indexing.Conveyor;
import frc.robot.subsystems.indexing.Gateway;
import frc.robot.Util;
import frc.robot.commands.shooting.AlignToTarget;
import frc.robot.commands.shooting.FullShootRoutine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousRoutine extends SequentialCommandGroup {
  
  /**
   * Creates a new AutonomousRoutine.
   */
  public AutonomousRoutine(Drivetrain drive, Limelight limelight, Shooter shooter, Conveyor conveyor, Gateway gateway, Intake intake) {
    // SequentialCommandGroup
    // note: I avoid using super() because it just calls addCommands and doesnt
    // allow me to reference member variables
    super();

    int alreadyShot = shooter.getBallsShot();

    addCommands(
      // drop intake
      new InstantCommand(() -> intake.dropBar(), intake),

      // back up a lil bit (really doesnt have to be THAT precise, this is fine...)
      new ArcadeDrive(drive, /*fwd/back*/() -> -0.5, /*turn*/() -> 0).withTimeout(1.5),

      new ConditionalCommand(
        // if no target is present: seek a target
        new SequentialCommandGroup(
          new ArcadeDrive(drive, /*fwd/back*/() -> 0, /*turn*/() -> .5).withTimeout(2),
          new ArcadeDrive(drive, /*fwd/back*/() -> 0, /*turn*/() -> -.5).withTimeout(4)
        ).withInterrupt(() -> limelight.isValidTargetPresent()),

        // else, do nothing
        new InstantCommand(),

        () -> !limelight.isValidTargetPresent()
      ),

      new ConditionalCommand(
        // if target present: auto align, shoot at target
        new SequentialCommandGroup(
          new AlignToTarget(limelight, drive, false, () -> 0).withTimeout(4), // if its not good after 5 seconds, yolo it!
          new FullShootRoutine(shooter, conveyor, gateway, () -> Util.calculateRPM(limelight.getApproximateDistanceMeters()))
            .withTimeout(10)
            .withInterrupt(() -> shooter.getBallsShot() - alreadyShot >= 3) // full trust in big man xuru
        ),

        // else, do nothing
        new InstantCommand(),
        
        () -> limelight.isValidTargetPresent()
      )
    );
  }
}
