/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexing.ConveyorBelt;
import frc.robot.subsystems.HoodedShooter;
import frc.robot.subsystems.indexing.SideBelt;
import frc.robot.subsystems.indexing.Gateway;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FullShootRoutine extends ParallelRaceGroup {
  /**
   * Full shooting routine
   */
  public FullShootRoutine(HoodedShooter shooter, SideBelt sideBelt, ConveyorBelt conveyor, Gateway Gateway, DoubleSupplier rpm) {

    // ParallelRaceGroup
    super(
      new FeedToGateway(sideBelt),        // the WHOLE time, feed to the stop-wheel
      new SequentialCommandGroup(           // then, in order:
        new SpinUpToSpeed(shooter, rpm),    // 1. spin up to the target RPM 
        new ParallelRaceGroup(              // 2. at the same time until a shot is made:
          new LoadToConveyor(Gateway),    //    - load from the stop-wheel into the conveyor
          new FeedToShooter(conveyor),      //    - feed from the conveyor to the shooter
          new SpinUntilShot(shooter, rpm)   //    - spin at the target RPM
        )
      )
    );
  }
}
