/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.indexing.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.indexing.Gateway;
import frc.robot.commands.shooting.Spin;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FullShootRoutine extends ParallelCommandGroup {

  private final Shooter m_shooter;
  private int m_ballsToShoot = Integer.MAX_VALUE;
  private int m_ballsShot = 0;

  /**
   * Full shooting routine
   */
  public FullShootRoutine(Shooter shooter, Conveyor conveyor, Gateway gateway, DoubleSupplier rpm) {

    super(
      new Spin(shooter, rpm, true), // spin to target rpm (but when done, stop the shooter)
      new FeedToShooter(conveyor),  // and spin conveyor
      new OpenGateway(gateway, () -> shooter.getClosedLoopErrorRPM() <= 100)
        // load balls to conveyor for shooting when the shooter is at the speed desired
    );

    m_shooter = shooter;
  }

  public FullShootRoutine(Shooter shooter, Conveyor conveyor, Gateway gateway, DoubleSupplier rpm, int ballsToShoot) {
    this(shooter, conveyor, gateway, rpm);

    m_ballsToShoot = ballsToShoot;
  }
}
