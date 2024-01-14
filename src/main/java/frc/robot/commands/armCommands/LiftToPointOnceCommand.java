
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.GeneralFunctions.*;
import static frc.robot.Constants.LiftConstants.*;
import frc.robot.subsystems.LiftSubsystem;

/** An example command that uses an example subsystem. */
 public class LiftToPointOnceCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LiftSubsystem m_subsystem;
  private double targetPosition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LiftToPointOnceCommand(LiftSubsystem subsystem, double target) {
    m_subsystem = subsystem;
    targetPosition = target;
     
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setSetPoint(targetPosition); 
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  
  }

  // Called once the command ends or is interrupted.
  // @Override
  public void end(boolean interrupted) {
    m_subsystem.setTarget(targetPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return allowedError(targetPosition, m_subsystem.getEncoderPosition(), TOLERANCE);
  }
}
