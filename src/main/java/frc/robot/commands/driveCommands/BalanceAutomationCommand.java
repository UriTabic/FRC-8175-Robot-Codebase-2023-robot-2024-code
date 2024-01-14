// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class BalanceAutomationCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  private final PIDController anglePidController;

  private final double KP = 0.4/18;
  private final double KI = 0.01/18;
  private final double KD = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BalanceAutomationCommand(DriveSubsystem subsystem) {
    driveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    anglePidController = new PIDController(KP, KI, KD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePidController.reset();
    anglePidController.setSetpoint(0);
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveSubsystem.arcadeDrive(-Math.copySign(anglePidController.calculate(driveSubsystem.getPitchAngle()), driveSubsystem.getPitchAngle()), 0);

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return allowedError(driveSubsystem.getPitchAngle(), 0, 0.03);
    return false;
  }
}
