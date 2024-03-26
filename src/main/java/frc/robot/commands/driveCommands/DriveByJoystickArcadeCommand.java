package frc.robot.commands.driveCommands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class DriveByJoystickArcadeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final DoubleSupplier fwdSup;
  private final DoubleSupplier rotSup;
  private final BooleanSupplier slowSup;
  private double fwd = 0;
  private double rot = 0;
  private boolean reverse = false;
  private boolean slow = false;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveByJoystickArcadeCommand(DriveSubsystem subsystem, DoubleSupplier fwdSup, DoubleSupplier rotSup, BooleanSupplier slowSup) {
    m_subsystem = subsystem;
    this.fwdSup = fwdSup;
    this.rotSup = rotSup;
    this.slowSup = slowSup;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    reverse = false;
  }



// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    fwd = -fwdSup.getAsDouble();
    rot = rotSup.getAsDouble();

    
    // if (RobotContainer.driveJoystick.getRawButtonReleased(RIGHT_JOYSTICK_CLICK)){
    //   reverse = !reverse;
    // }

    if(fwd < -0.7){
      fwd = -1;
    }
    if(fwd > 0.7){
      fwd = 1;
    }
    
    if (slowSup.getAsBoolean()){
      slow = !slow;
    }
    
    
    if (reverse) {
      fwd = -fwd;
    }

    if(slow){
      fwd /= 1.3;
    }
    fwd /= 2;
    rot /=2;
    SmartDashboard.putNumber("fwd", fwd);
    SmartDashboard.putNumber("rot", rot);
    
    m_subsystem.arcadeDrive(fwd, rot);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
