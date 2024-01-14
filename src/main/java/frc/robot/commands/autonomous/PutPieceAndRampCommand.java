package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LiftConstants;
import frc.robot.commands.armCommands.LiftToPointOnceCommand;
import frc.robot.commands.driveCommands.BalanceAutomationCommand;
import frc.robot.commands.driveCommands.DriveBackAndEject;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class PutPieceAndRampCommand extends SequentialCommandGroup {
    public PutPieceAndRampCommand(DriveSubsystem drive,GripperSubsystem gripper, LiftSubsystem lift)
    {
        addCommands(
            new LiftToPointOnceCommand(lift, LiftConstants.CUBE_DOWN),
            //new WaitCommand(3),
            new DriveBackAndEject(drive, gripper).withTimeout(1.8),
            new LiftToPointOnceCommand(lift, LiftConstants.FOLD),
            new BalanceAutomationCommand(drive)
        );
    }
}