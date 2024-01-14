// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.GripperConstants.*;

public class GripperSubsystem extends PomSubsystem {

    CANSparkMax gripperMotor;
    
  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    gripperMotor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
    gripperMotor.setIdleMode(IdleMode.kBrake);
       
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  /** sets the speed of the gripper
   * @param speed the speed of the gripper you want
   */
  @Override
  public void setMotor(double speed)
  {
    gripperMotor.set(speed);
  }
}
