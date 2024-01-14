package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LiftConstants;
import frc.robot.commands.armCommands.LiftToPointOnceCommand;
import frc.robot.commands.driveCommands.DriveBackAndEject;
import frc.robot.commands.driveCommands.DriveConstantSpeedCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class PutCubeUpAndOutCommand extends SequentialCommandGroup {
    public PutCubeUpAndOutCommand(DriveSubsystem drive,GripperSubsystem gripper, LiftSubsystem lift)
    {
        addCommands(
            new LiftToPointOnceCommand(lift, LiftConstants.CUBE_UP).withTimeout(3),
            new WaitCommand(2),
            new DriveBackAndEject(drive, gripper).withTimeout(1.4),
            new LiftToPointOnceCommand(lift, LiftConstants.FOLD),
            new DriveConstantSpeedCommand(drive, -2.5).withTimeout(3)
        );
    }
}