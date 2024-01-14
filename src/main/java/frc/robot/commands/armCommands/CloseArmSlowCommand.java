

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.LiftConstants.*;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.LiftSubsystem;


 public class CloseArmSlowCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final BooleanSupplier finish;
  private final LiftSubsystem m_subsystem;

  /**
   * Creates a new CloseArmSlowCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CloseArmSlowCommand(LiftSubsystem subsystem, BooleanSupplier finish) {
    m_subsystem = subsystem;
    this.finish = finish;
     
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setMotor(CLOSE_SLOW);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotor(0);
    m_subsystem.setTarget(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isFoldSwitchPressed() || finish.getAsBoolean();
  }
}
