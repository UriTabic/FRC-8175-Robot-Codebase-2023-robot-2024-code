package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.GripperConstants.*;
import frc.robot.commands.gripperCommands.GripperOnceCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class DriveBackAndEject extends ParallelCommandGroup{
    public DriveBackAndEject(DriveSubsystem drive, GripperSubsystem gripper){
        addCommands(
            new DriveConstantSpeedCommand(drive, DRIVE_BACK),
                        new GripperOnceCommand(gripper, EJECT)
        );
    }
    
}
