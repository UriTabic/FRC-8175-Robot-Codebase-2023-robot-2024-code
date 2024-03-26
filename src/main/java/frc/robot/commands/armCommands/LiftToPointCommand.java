
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import static frc.robot.Constants.LiftConstants.*;
import static frc.robot.Constants.LiftConstants.FIDDER;
import static frc.robot.Constants.LiftConstants.FOLD;
import static frc.robot.Constants.LiftConstants.GROUND;
import static frc.robot.Constants.LiftConstants.NONE;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GeneralFunctions;
import frc.robot.subsystems.LiftSubsystem;

/** An example command that uses an example subsystem. */
 public class LiftToPointCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LiftSubsystem m_subsystem;
  private final BooleanSupplier groundSup;
  private final BooleanSupplier foldSup;
  private final BooleanSupplier cubeDownSup;
  private final BooleanSupplier fidderSup;
  private final BooleanSupplier upSup;
  private final BooleanSupplier downSup;
  private double targetPosition = 0;
  private double lastSpeed = 0;
  Timer timer = new Timer();
  double MAX_ACCELERATION = 0.04;
  // ProfiledPIDController controller;
  // TrapezoidProfile.State goal;
  PIDController pid = new PIDController(PROFILE_KP, KI, KD);


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LiftToPointCommand(LiftSubsystem subsystem, BooleanSupplier groundSup, BooleanSupplier foldSup, BooleanSupplier fidderSup, BooleanSupplier cubeDownSup, BooleanSupplier upSup, BooleanSupplier downSup) {
    m_subsystem = subsystem;
     this.groundSup = groundSup;
     this.foldSup = foldSup;
     this.cubeDownSup = cubeDownSup;
     this.fidderSup = fidderSup;
     this.upSup = upSup;
     this.downSup = downSup;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  
    if(groundSup.getAsBoolean())
    {
      targetPosition = GROUND;
    }
    if(foldSup.getAsBoolean())
    {
      targetPosition = FOLD;
    }
    if(cubeDownSup.getAsBoolean())
    {
      targetPosition = CUBE_DOWN;
    }
    if(fidderSup.getAsBoolean())
     {
      targetPosition = FIDDER;
    }
    if(m_subsystem.getTarget() != NONE)
    {
      targetPosition = m_subsystem.getTarget();
      m_subsystem.setTarget(NONE);
    }
    if(upSup.getAsBoolean())
    {
        targetPosition += 1;
    }
    if(downSup.getAsBoolean())
    {
        targetPosition -= 1;
    }
    if((m_subsystem.getEncoderPosition() >= GROUND - 1 && targetPosition == GROUND) || (m_subsystem.getEncoderPosition() <= FOLD + 1 && targetPosition == FOLD)){
      m_subsystem.setMotor(0);
    }
    else if((m_subsystem.getEncoderPosition() > GROUND - 10 && targetPosition == GROUND) || (m_subsystem.getEncoderPosition() < FOLD + 8 && targetPosition == FOLD))
    {
      m_subsystem.setMotor(0.05 * (targetPosition == GROUND ? 1 : -1));
    }
    else {
      pid.setSetpoint(targetPosition);
      double vel = pid.calculate(m_subsystem.getEncoderPosition());
      
      if(!GeneralFunctions.allowedError(vel, lastSpeed, MAX_ACCELERATION)){
        vel = lastSpeed + (MAX_ACCELERATION * Math.copySign(1, vel- lastSpeed));
      }
      if(vel > MAX_VEL){
        vel = MAX_VEL;
      }
      else if(vel < -MAX_VEL){
        vel = -MAX_VEL;
      }
      SmartDashboard.putNumber("vel", vel);
      
      lastSpeed = vel;
      m_subsystem.setMotor(vel);

    }

    SmartDashboard.putNumber("target position", targetPosition);
    
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
