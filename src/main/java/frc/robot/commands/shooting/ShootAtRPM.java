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
import frc.robot.subsystems.indexing.StopWheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAtRPM extends ParallelRaceGroup {
  /**
   * Creates a new Shoot.
   */
  public ShootAtRPM(HoodedShooter shooter, SideBelt sideBelt, ConveyorBelt conveyor, StopWheel stopWheel, DoubleSupplier rpm) {

    // ParallelRaceGroup
    super(
      new FeedToStopWheel(sideBelt),        // the WHOLE time, feed to the stop-wheel
      new SequentialCommandGroup(
        new SpinUpToSpeed(shooter, rpm),    // spin up to the target RPM before anything
        new ParallelRaceGroup(
          new LoadToConveyor(stopWheel),    // load from the stop-wheel into the conveyor
          new FeedToShooter(conveyor),      // feed from the conveyor to the shooter
          new SpinUntilShot(shooter, rpm)   // spin at the target RPM until a shot is made, in which case end everything
        )
      )
    );
  }
}
