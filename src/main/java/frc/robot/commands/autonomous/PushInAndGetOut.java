package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.DriveConstantSpeedCommand;
import frc.robot.subsystems.DriveSubsystem;

public class PushInAndGetOut extends SequentialCommandGroup{
    private DriveSubsystem mDrive;
    public PushInAndGetOut(DriveSubsystem drive)
    {
        mDrive = drive;
        

        addCommands(
            new WaitCommand(0.2),
             new DriveConstantSpeedCommand(mDrive, 1.5).withTimeout(2),
            // new TurnByDegreeCommand(mDrive, 180),
            // new DriveConstantSpeedCommand(mDrive, -0.5).withTimeout(0.5)
             new DriveConstantSpeedCommand(mDrive, -2).withTimeout(5)
        );
    }
    
}
