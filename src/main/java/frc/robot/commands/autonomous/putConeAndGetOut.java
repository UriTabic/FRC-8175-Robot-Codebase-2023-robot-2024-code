package frc.robot.commands.autonomous;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.LiftConstants;
import static frc.robot.Constants.DriveConstants.*;

import frc.robot.commands.armCommands.LiftToPointOnceCommand;
import frc.robot.commands.driveCommands.DriveConstantSpeedCommand;
import frc.robot.commands.gripperCommands.GripperOnceCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class putConeAndGetOut extends SequentialCommandGroup{
    private DriveSubsystem mDrive;
    private GripperSubsystem mGripper;
    private LiftSubsystem mLift;
    private TrajectoryFactory mTrajectoryFactory;
    public putConeAndGetOut(DriveSubsystem drive, GripperSubsystem gripper, LiftSubsystem lift, TrajectoryFactory factory)
    {
        mDrive = drive;
        mGripper = gripper;
        mLift = lift;
        mTrajectoryFactory = factory;

        addCommands(
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new LiftToPointOnceCommand(mLift, LiftConstants.CONE),
                new WaitCommand(2)
            ),
            new ParallelDeadlineGroup(
                new DriveConstantSpeedCommand(mDrive, 0.5).withTimeout(0.8),
                new GripperOnceCommand(mGripper,GripperConstants.EJECT)
            ),
            new ParallelCommandGroup(
                new LiftToPointOnceCommand(mLift, LiftConstants.FOLD),
                new WaitCommand(1.2)
            ),
            new RamseteCommand(mTrajectoryFactory.trajectoryFactory(
                                                                mDrive.getPose(),
                                                                new ArrayList<Translation2d>(),
                                                                new Pose2d(10,1,Rotation2d.fromDegrees(90)),
                                                                true),
                                                                mDrive::getPose,
                                                                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                                                                DRIVE_KINEMATICS,
                                                                mDrive::tankDriveVolts,
                                                                mDrive),
            new RamseteCommand(mTrajectoryFactory.trajectoryFactory(
                                                                    mDrive.getPose(),
                                                                    new ArrayList<Translation2d>(),
                                                                    new Pose2d(10,1,Rotation2d.fromDegrees(270)),
                                                                    false),
                                                                    mDrive::getPose,
                                                                    new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                                                                    DRIVE_KINEMATICS,
                                                                    mDrive::tankDriveVolts,
                                                                    mDrive)
        );
    }
    
}
