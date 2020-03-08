/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexing.ConveyorBelt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.indexing.Gateway;
import frc.robot.commands.shooting.Spin;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FullShootRoutine extends SequentialCommandGroup {
  /**
   * Full shooting routine
   */
  public FullShootRoutine(Shooter shooter, ConveyorBelt conveyor, Gateway gateway, DoubleSupplier rpm) {

    // super(
    //   new SpinUpToSpeed(shooter, rpm),    // 1. spin up to the target RPM 
    //   new ParallelRaceGroup(              // 2. at the same time until a shot is made:
    //     new SpinUntilShot(shooter, rpm),  //    - spin at the target RPM
    //     new LoadToConveyor(gateway),      //    - load from the gateway into the conveyor
    //     new FeedToShooter(conveyor)       //    - feed from the conveyor to the shooter
    //   )
    // );

    super(
      new Spin(shooter, rpm).withInterrupt(() -> Math.abs(shooter.getClosedLoopError()) <= 200),
      new ParallelCommandGroup(
        new Spin(shooter, rpm),
        new FeedToShooter(conveyor),
        new LoadToConveyor(gateway)
          .withInterrupt(() -> Math.abs(shooter.getClosedLoopError()) >= 210)
      ).withInterrupt(() -> Math.abs(shooter.getClosedLoopError()) >= 400)
    );
  }
}
