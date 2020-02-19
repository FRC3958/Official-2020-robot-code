/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class Limelight extends SubsystemBase {

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");;
  
  private final NetworkTableEntry m_tv = m_table.getEntry("tv");
  private final NetworkTableEntry m_tx = m_table.getEntry("tx");
  private final NetworkTableEntry m_ty = m_table.getEntry("ty");


  private final AHRS m_ahrs = new AHRS(SPI.Port.kMXP);
  private double m_absoluteTargetAngleX = 0.0;

  /**
   * Creates a new Limelight.
   */
  public Limelight() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isValidTargetPresent()) {

      m_absoluteTargetAngleX = m_ahrs.getYaw() + getAngleOffsetX();
    }
  }

  public boolean isValidTargetPresent() {

    return m_tv.getBoolean(false);
  }

  /**
   * Horizontal angle offset to target
   * @return
   */
  public double getAngleOffsetX() {

    return m_tx.getDouble(0.0);
  }

  /**
   * Vertical angle offset to target
   * @return
   */
  public double getAngleOffsetY() {

    return m_ty.getDouble(0.0);
  }

  public double getAngleOffsetXFromMemory() {

    return m_absoluteTargetAngleX - m_ahrs.getYaw();
  }

  public double getBestAngleOffsetX() {

    return isValidTargetPresent() ? getAngleOffsetX() : getAngleOffsetXFromMemory();
  }

  public double getApproximateDistance() {

    // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
    // d = (h2-h1) / tan(a1+a2)

    return (FieldConstants.kOuterPortCenterHeightMeters - VisionConstants.kLimelightMountHeightMeters)
      / Math.tan(Units.degreesToRadians(VisionConstants.kLimelightMountAngleDeg + getAngleOffsetY()));
  }
}