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
import static frc.robot.constants.Field.*;
import static frc.robot.constants.VisionConstants.*;

public class Limelight extends SubsystemBase {

  public enum LedMode {

    kAuto(0),
    kForceOff(1),
    kBlink(2),
    kForceOn(3);

    private final int value;
    private LedMode(int value) {
      this.value = value;
    }

    public int getInt() {
      return value;
    }
  }

   public enum CamMode {

    kVisionProcessor(0),
    kDriver(1);

    private final int value;
    private CamMode(int value) {
      this.value = value;
    }

    public int getInt() {
      return value;
    }
  }
  
  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");;
  
  private final NetworkTableEntry m_tv = m_table.getEntry("tv");
  private final NetworkTableEntry m_tx = m_table.getEntry("tx");
  private final NetworkTableEntry m_ty = m_table.getEntry("ty");
  private final NetworkTableEntry m_ledMode =  m_table.getEntry("ledMode");
  private final NetworkTableEntry m_camMode  = m_table.getEntry("camMode");

  private final AHRS m_ahrs = new AHRS(SPI.Port.kMXP);

  private final LinearFilter m_yAngleFilter = LinearFilter.movingAverage(3);
  private double m_lastYAngle = 0.0;

  private final LinearFilter m_distanceFilter = LinearFilter.movingAverage(3);
  private double m_lastFilteredDistance = 0;

  private double m_absoluteTargetAngleX = 0.0;
  private double m_bestTargetAngleX = 0.0;

  /**
   * Creates a new Limelight.
   */
  public Limelight() {

  }

  public void setCamMode(CamMode mode){

    m_camMode.setDouble(mode.getInt());
  }

  //led mode 1 meaning its off, 2 meaning its blinking, and 3 meaning its on
  public void setLedMode(LedMode mode){

    m_ledMode.setDouble(mode.getInt());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    updateAbsoluteAngle();
    updateAngleOffsetY();
    updateBestAngle();
    updateFilteredDistance();

    SmartDashboard.putBoolean("Valid target", isValidTargetPresent());
    SmartDashboard.putNumber("Offset X deg (raw)", getAngleOffsetX());
    SmartDashboard.putNumber("Offset X deg (interp)", getBestAngleOffsetX());
    SmartDashboard.putNumber("Distance estimate", getApproximateDistanceMeters());
  }

  public boolean isValidTargetPresent() {

    return m_tv.getBoolean(false) || getAngleOffsetX() != 0.0;
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

  public void updateAngleOffsetY() {

    m_lastYAngle = isValidTargetPresent() ? m_yAngleFilter.calculate(getAngleOffsetYRaw()) : m_lastYAngle;
  }
 
  public double getAngleOffsetY() {

    return m_lastYAngle;
  }

  public void updateBestAngle() {
    m_bestTargetAngleX = isValidTargetPresent() ? getAngleOffsetX() : getAngleOffsetXFromMemory();
  }

  public void updateAbsoluteAngle() {

    if(isValidTargetPresent())
      m_absoluteTargetAngleX = m_ahrs.getYaw() + getAngleOffsetX();
  }

  public double getAngleOffsetXFromMemory() {

    return m_absoluteTargetAngleX - m_ahrs.getYaw();
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

    m_lastFilteredDistance = m_distanceFilter.calculate(getApproximateDistanceMetersRaw());
  }

  public double getApproximateDistanceMeters() {

    return m_lastFilteredDistance;
  }
}
