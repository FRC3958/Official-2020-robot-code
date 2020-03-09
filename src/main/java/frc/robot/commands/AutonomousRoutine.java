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
    super(
      
      // drop intake
      new InstantCommand(() -> intake.dropBar(), intake),

      // back up a lil bit (really doesnt have to be THAT precise, this is fine...)
      new ArcadeDrive(drive, /*fwd/back*/() -> -0.5, /*turn*/() -> 0).withTimeout(1.5),

      // ensure we see vision target
      new ConditionalCommand(
        /*true: nothing... blank command*/ new InstantCommand(),
        
        /*false: seek target*/
        new SequentialCommandGroup(
          new ArcadeDrive(drive, /*fwd/back*/() -> 0, /*turn*/() -> .5).withTimeout(2),
          new ArcadeDrive(drive, /*fwd/back*/() -> 0, /*turn*/() -> -.5).withTimeout(4)
        ).withInterrupt(() -> limelight.isValidTargetPresent()),

        () -> limelight.isValidTargetPresent()
      ),

      // if vision target is present, shoot at it
      new ConditionalCommand(
        /*true: auto align, shoot to target*/
        new SequentialCommandGroup(
          new AlignToTarget(limelight, drive, false, () -> 0).withTimeout(5), // if its not good after 5 seconds, yolo it!
          new FullShootRoutine(shooter, conveyor, gateway, () -> Util.calculateRPM(limelight.getApproximateDistanceMeters()))
            .withTimeout(10) // full trust in big man xuru
        ),
        /*false: we hath failed! do nothing and be sad*/ new InstantCommand(),
        
        () -> limelight.isValidTargetPresent())
    );
  }
}
