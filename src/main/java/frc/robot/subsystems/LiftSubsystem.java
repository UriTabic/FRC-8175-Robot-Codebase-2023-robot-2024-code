
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static frc.robot.Constants.LiftConstants.FOLD_MICRO_SWITCH_ID;
import static frc.robot.Constants.LiftConstants.GROUND;
import static frc.robot.Constants.LiftConstants.GROUND_MICRO_SWITCH_ID;
import static frc.robot.Constants.LiftConstants.KD;
import static frc.robot.Constants.LiftConstants.KI;
import static frc.robot.Constants.LiftConstants.KP;
import static frc.robot.Constants.LiftConstants.MOTOR_ID;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiftSubsystem extends PomSubsystem {

    private final CANSparkMax liftMotor;
    private SparkPIDController pid;
    private DigitalInput foldMicroSwitch;
    private DigitalInput groundMicroSwitch;

    private double targetPosition = -1;

    //ShuffleboardTab liftTab = Shuffleboard.getTab("lift");
    
    
  /** Creates a new LiftSubsystem. */
  public LiftSubsystem() {

    liftMotor = new CANSparkMax(MOTOR_ID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    pid = liftMotor.getPIDController();
    
    
    pid.setP(KP);
    pid.setI(KI);
    pid.setD(KD);

    foldMicroSwitch = new DigitalInput(FOLD_MICRO_SWITCH_ID);
    groundMicroSwitch = new DigitalInput(GROUND_MICRO_SWITCH_ID);

    SmartDashboard.putNumber("arm encoder", liftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("arm motor", liftMotor.get());

    

  }

  @Override
  public void periodic() {
    //liftTab.add("arm encoder", liftMotor.getEncoder().getPosition()).withPosition(0, 0).withSize(1, 1).withWidget(BuiltInWidgets.kNumberSlider);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", liftMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("arm fold limit switch", isFoldSwitchPressed());
    SmartDashboard.putBoolean("arm ground limit switch", isGroundSwitchPressed());
    SmartDashboard.putNumber("arm motor", liftMotor.get());
    if(isFoldSwitchPressed())
    {
      resetEncoder();
    }
    if(isGroundSwitchPressed())
    {
      liftMotor.getEncoder().setPosition(GROUND);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /** returns is the fold switch pressed
   * @return the fold limit switch value
   */
  public boolean isFoldSwitchPressed(){
    return !foldMicroSwitch.get();
  }

    /** returns is the ground switch pressed
   * @return the ground limit switch value
   */
  public boolean isGroundSwitchPressed(){
    return !groundMicroSwitch.get();
  }

  /** Resets the encoder to currently read a position of 0. */
  @Override
  public void resetEncoder() {
    liftMotor.getEncoder().setPosition(0);
  }

  /** returns the value of the alternate encoder
   * @return the encoder position
   */
  @Override
  public double getEncoderPosition()
  {
    return liftMotor.getEncoder().getPosition();
  }

  public RelativeEncoder getEncoder(){
    return liftMotor.getEncoder();
  }

  /** set the motors to go to a specified position.
   * @param targetHeight the position to go to
   */
  @Override
  public void setSetPoint(double target) {
    pid.setReference(target, CANSparkMax.ControlType.kPosition);
  }

  

  /**sets the motor to a paramater value
  * @param power the power to set the motor to
  */
  @Override
  public void setMotor(double speed)
  {
    liftMotor.set(speed);
  }

  /** stops the motor */
  public void offMotors()
  {
    liftMotor.set(0);
  }

  /**
   * returns the current target the motors needs to go to. -1 means none
   * @return the target position
   */
  public double getTarget()
  {
    return targetPosition;
  }

  /**
   * sets the current target the motors needs to go to. -1 means none
   * @param target the target
   */
  public void setTarget(double target)
  {
    targetPosition = target;
  }

  public CANSparkMax getMotor(){
    return liftMotor;
  }
}
