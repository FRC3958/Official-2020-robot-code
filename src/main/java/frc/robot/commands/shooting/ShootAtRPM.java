/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.HoodedShooter;
import frc.robot.subsystems.Indexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAtRPM extends SequentialCommandGroup {
  /**
   * Creates a new Shoot.
   */
  public ShootAtRPM(HoodedShooter shooter, Indexer indexer, Feeder feeder, DoubleSupplier rpm) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new SpinUpToSpeed(shooter, rpm), 
      new ParallelCommandGroup(
        new LoadBall(indexer).withTimeout(0.2),
        new FeedBallUntilShot(feeder, shooter),
        new SpinUntilShot(shooter, rpm) 
      )
    );
  }
}
