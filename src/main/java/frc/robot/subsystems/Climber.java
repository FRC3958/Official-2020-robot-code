/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.Constants.climberconstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;


public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  //TODO: find heights of each climber level in Talon units

  public WPI_TalonFX m_down;
  public WPI_TalonSRX m_up;

  public Climber() {
    m_down = new WPI_TalonFX(climberconstants.kdown);
    m_up = new WPI_TalonSRX(climberconstants.kup);

    m_up.configFactoryDefault();
    m_down.configFactoryDefault();

    m_up.setNeutralMode(NeutralMode.Brake);
    m_down.setNeutralMode(NeutralMode.Brake);

    m_up.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,climberconstants.kPrimaryPIDLoopIdx,Constants.kTimeout);

    m_up.setSensorPhase(false);

    //TODO: tune climber PID for up and down so that gravity is not a problem
     //up
     m_up.config_kF(0,climberconstants.Kclimberup.kF);
     m_up.config_kP(0, climberconstants.Kclimberup.kP);
     m_up.config_kI(0, climberconstants.Kclimberup.kI);
     m_up.config_kD(0, climberconstants.Kclimberup.kD);
     
     //down
     m_up.config_kF(1, climberconstants.Kclimberdown.kF);
     m_up.config_kP(1, climberconstants.Kclimberdown.kP);
     m_up.config_kI(1, climberconstants.Kclimberdown.kI);
     m_up.config_kD(1, climberconstants.Kclimberdown.kD);
     
    //TODO: Put other things related to motionmagic



  }
  public double GetCurrentPosition(){
    return m_up.getSelectedSensorPosition();
  }

  public void manualcontrol(double speedo, double speedt){

    m_up.set(ControlMode.PercentOutput,speedo);
    m_down.set(ControlMode.PercentOutput,speedt);
  }

  public void gototarget (double target){
    //1000 is just a placeholder
    if(Math.abs(target-this.GetCurrentPosition())>1000){
      if(target > this.GetCurrentPosition()){
        m_up.selectProfileSlot(0, 0);
     }
     else{
       m_up.selectProfileSlot(1, 0);
     }
    }
    m_up.set(ControlMode.MotionMagic, target);
  

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
