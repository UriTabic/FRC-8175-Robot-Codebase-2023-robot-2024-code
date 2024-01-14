package frc.robot.commands.positioningCommands;

import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.autonomous.TrajectoryFactory;
import frc.robot.subsystems.DriveSubsystem;

public class ConePositioningCommand extends Command {
    private RamseteCommand command;
    private DriveSubsystem mDriveSubsystem;
    private TrajectoryFactory mFactory;
    public ConePositioningCommand(DriveSubsystem drive, TrajectoryFactory factory){
        mDriveSubsystem = drive;
        mFactory = factory;
    }
    // @Override
    // public void initialize(){
    //     command = new RamseteCommand(mFactory.trajectoryFactory(
    //         mDriveSubsystem.getPose(),
    //         new ArrayList<Translation2d>(),
    //         mDriveSubsystem.getPoseTarget(AutoMode.kCone),
    //         false),
    //     mDriveSubsystem::getPose,
    //     new RamseteController(RAMSETE_B, RAMSETE_ZETA),
    //     new SimpleMotorFeedforward(
    //         KS_VOLTS,
    //         KV_VOLT_SECOND_PER_METER,
    //         KA_VOLT_SECONDS_SQUARE_PER_METER),
    //     DRIVE_KINEMATICS,
    //     mDriveSubsystem::getWheelSpeeds,
    //     new PIDController(KP_DRIVE_VEL, 0, 0),
    //     new PIDController(KP_DRIVE_VEL, 0, 0),
    //     mDriveSubsystem::tankDriveVolts,
    //     mDriveSubsystem);
    //     command.schedule();
        
    // }
    // @Override
    // public boolean isFinished()
    // {
    //     return true;
    // }
}
