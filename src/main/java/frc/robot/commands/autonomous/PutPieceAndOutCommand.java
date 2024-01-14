package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LiftConstants;
import frc.robot.commands.armCommands.LiftToPointOnceCommand;
import frc.robot.commands.driveCommands.DriveBackAndEject;
import frc.robot.commands.driveCommands.DriveConstantSpeedCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class PutPieceAndOutCommand extends SequentialCommandGroup {
    public PutPieceAndOutCommand(DriveSubsystem drive,GripperSubsystem gripper, LiftSubsystem lift)
    {
        addCommands(
            new LiftToPointOnceCommand(lift, LiftConstants.CUBE_DOWN).withTimeout(5),
            //new WaitCommand(3),
            new DriveBackAndEject(drive, gripper).withTimeout(2),
            new LiftToPointOnceCommand(lift, LiftConstants.FOLD),
            new DriveConstantSpeedCommand(drive, -2).withTimeout(3)
        );
    }
}