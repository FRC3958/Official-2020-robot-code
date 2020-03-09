/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.indexing.ConveyorBelt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.indexing.Gateway;
import frc.robot.commands.shooting.Spin;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FullShootRoutine extends ParallelCommandGroup {
  /**
   * Full shooting routine
   */
  public FullShootRoutine(Shooter shooter, ConveyorBelt conveyor, Gateway gateway, DoubleSupplier rpm) {

    super(
      new Spin(shooter, rpm, true), // spin to target rpm (but when done, stop the shooter)
      new FeedToShooter(conveyor),  // and spin conveyor
      new OpenGateway(gateway, () -> shooter.getClosedLoopErrorRPM() <= 100)
        // load balls to conveyo for shooting when the shooter is at the speed desired
    );
  }
}
