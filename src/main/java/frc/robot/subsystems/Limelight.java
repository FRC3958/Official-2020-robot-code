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
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.constants.Field.*;
import static frc.robot.constants.VisionConstants.*;

public class Limelight extends SubsystemBase {

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");;
  
  private final NetworkTableEntry m_tv = m_table.getEntry("tv");
  private final NetworkTableEntry m_tx = m_table.getEntry("tx");
  private final NetworkTableEntry m_ty = m_table.getEntry("ty");
  private final NetworkTableEntry m_ledMode =  m_table.getEntry("ledMode");
  private final NetworkTableEntry m_camMode  = m_table.getEntry("camMode");

  private final AHRS m_ahrs = new AHRS(SPI.Port.kMXP);

  private final LinearFilter m_yAngleFilter = LinearFilter.movingAverage(5);
  private double m_lastFilteredYOffset = 0.0;

  private final LinearFilter m_distanceFilter = LinearFilter.movingAverage(7);
  private double m_lastFilteredDistance = 0;

  private double m_absoluteTargetAngleX = 0.0;
  private double m_bestTargetAngleX = 0.0;

  private final Timer m_accessTimer = new Timer();

  /**
   * Creates a new Limelight.
   */
  public Limelight() {

    m_accessTimer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    updateAbsoluteAngle();
    updateFilteredOffsetY();
    updateBestAngle();
    updateFilteredDistance();
    updateLights();

    SmartDashboard.putNumber("LED Timer", m_accessTimer.get());
    SmartDashboard.putBoolean("Valid target", isValidTargetPresent());
    SmartDashboard.putNumber("Offset X deg (raw)", getAngleOffsetX());
    SmartDashboard.putNumber("Offset X deg (interp)", getBestAngleOffsetX());
    SmartDashboard.putNumber("Distance estimate", getApproximateDistanceMeters());
  }

  public void resetLedTimer() {

    m_accessTimer.reset();
  }
  
  private void updateLights() {

    if(m_accessTimer.get() > 0.5) {
      setLedMode(LedMode.kForceOff);
    } else {
      setLedMode(LedMode.kForceOn);
    }
  }

  private int getCamMode() {

    return (int)m_camMode.getDouble(0);
  }

  private int getLedMode() {

    return (int)m_ledMode.getDouble(0);
  }

  private void setCamMode(int mode){

    m_camMode.setDouble(mode);
  }

  private void setLedMode(int mode){

    m_ledMode.setDouble(mode);
  }

  public boolean isValidTargetPresent() {

    return m_tv.getDouble(0) < 1.0 || getAngleOffsetX() != 0.0;
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
  public double getAngleOffsetYRaw() {

    return m_ty.getDouble(0.0);
  }

  public void updateFilteredOffsetY() {

    if(isValidTargetPresent())
      m_lastFilteredYOffset = m_yAngleFilter.calculate(getAngleOffsetYRaw());
  }
 
  public double getAngleOffsetY() {

    return m_lastFilteredYOffset;
  }

  public void updateAbsoluteAngle() {

    if(isValidTargetPresent())
      m_absoluteTargetAngleX = m_ahrs.getYaw() + getAngleOffsetX();
  }

  public double getAngleOffsetXFromMemory() {

    return m_absoluteTargetAngleX - m_ahrs.getYaw();
  }

  public void updateBestAngle() {
    m_bestTargetAngleX = isValidTargetPresent() ? getAngleOffsetX() : getAngleOffsetXFromMemory();
  }

  public double getBestAngleOffsetX() {

    return m_bestTargetAngleX;
  }

  /**
   * Gets the HORIZONTAL distance to the target from back of frame in meters
   * @return
   */
  public double getApproximateDistanceMetersRaw() {

    // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
    // d = (h2-h1) / tan(a1+a2)

    return Math.abs((kOuterPortCenterHeightMeters - kLimelightMountHeightMeters)
      / Math.tan(Units.degreesToRadians(kLimelightMountAngleDeg + getAngleOffsetY()))
        - kLimelightMountDistanceFromBackMeters);
  }

  public void updateFilteredDistance() {

    if(isValidTargetPresent())
      m_lastFilteredDistance = m_distanceFilter.calculate(getApproximateDistanceMetersRaw());
  }

  public double getApproximateDistanceMeters() {

    return m_lastFilteredDistance;
  }
}
