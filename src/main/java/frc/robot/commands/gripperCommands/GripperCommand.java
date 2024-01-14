// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripperCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GripperSubsystem;
import static frc.robot.Constants.GripperConstants.*;

import java.util.function.BooleanSupplier;

/** An example command that uses an example subsystem. */
public class GripperCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final BooleanSupplier in;
  private final BooleanSupplier out;
  private final BooleanSupplier slowOut;
  private final GripperSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GripperCommand(GripperSubsystem subsystem, BooleanSupplier in, BooleanSupplier out, BooleanSupplier slowOut) {
    m_subsystem = subsystem;
    this.in = in;
    this.out = out;
    this.slowOut = slowOut;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.getDefaultCommand();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //changes the gripper power according to buttons pressed
    if(in.getAsBoolean()){
      m_subsystem.setMotor(TAKE_IN);
    }
    else if(slowOut.getAsBoolean()){
      m_subsystem.setMotor(EJECT_SLOW);
    }
    else if(out.getAsBoolean())
    {
      m_subsystem.setMotor(EJECT);
    }
    else
    {
      m_subsystem.setMotor(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
