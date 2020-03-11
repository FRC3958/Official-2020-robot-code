/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
public class Trenchauto extends SequentialCommandGroup {
  /**
   * Creates a new Trenchauto.
   */
  //TODO: get timeout and distance
  public Trenchauto(Drivetrain drivetrain, Limelight limelight, Shooter shooter, Conveyor conveyor, Gateway gateway, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
      super();
      addCommands();
        //drop intake
        /*new InstantCommand(() -> intake.dropBar(), intake),
        new ConditionalCommand(
          
          new ConditionalCommand(
          // shoot three balls at the target
        new SequentialCommandGroup(
          new AlignToTarget(limelight, drivetrain, false, () -> 0).withTimeout(4), 
          new FullShootRoutine(shooter, conveyor, gateway, () -> Util.calculateRPM(limelight.getApproximateDistanceMeters()))
            .withTimeout(10)
          
        ),
        //turn and go through and back on the trench
       new ArcadeDrive(drivetrain, () -> 0,() -> -.5).withTimeout(2),
        new ParallelCommandGroup(
        new ArcadeDrive(drivetrain,() -> 1.5, () -> 0).alongWith(intake.spin(0.6)
        )),
        new ArcadeDrive(drivetrain, () -> 0,() -> -.5).withTimeout(2),
        new ArcadeDrive(drivetrain, () -> 1.5,() -> 0).withTimeout(2),

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
      
    
    )

  );
    */
  }
}
