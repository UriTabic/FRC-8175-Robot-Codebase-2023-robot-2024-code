

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import static frc.robot.Constants.LiftConstants.GROUND;
import static frc.robot.Constants.LiftConstants.OPEN_RESET;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

/** An example command that uses an example subsystem. */
public class OpenArmSlowForResetCommand  extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LiftSubsystem m_subsystem;
  private final BooleanSupplier finish;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OpenArmSlowForResetCommand (LiftSubsystem subsystem, BooleanSupplier finish) {
    m_subsystem = subsystem;
    this.finish = finish;

     
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setMotor(OPEN_RESET);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotor(0);
    m_subsystem.setTarget(GROUND);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isGroundSwitchPressed() || finish.getAsBoolean();
  }
}
