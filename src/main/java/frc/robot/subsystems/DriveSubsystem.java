
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.DriveConstants.LEFT_ENCODER_IDS;
import static frc.robot.Constants.DriveConstants.LEFT_MOTOR1;
import static frc.robot.Constants.DriveConstants.LEFT_MOTOR2;
import static frc.robot.Constants.DriveConstants.RATE;
import static frc.robot.Constants.DriveConstants.RIGHT_ENCODER_IDS;
import static frc.robot.Constants.DriveConstants.RIGHT_MOTOR1;
import static frc.robot.Constants.DriveConstants.RIGHT_MOTOR2;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.MutableMeasure.mutable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveSubsystem extends PomSubsystem {

  /**the bot pose parameters from limelight */

  private double output = 0;

  private Field2d field;

  private final WPI_VictorSPX masterRightMotor = new WPI_VictorSPX(RIGHT_MOTOR1);
  private final WPI_TalonSRX slaveRightMotor = new WPI_TalonSRX(RIGHT_MOTOR2);

  private final WPI_VictorSPX masterLeftMotor = new WPI_VictorSPX(LEFT_MOTOR1);
  private final WPI_TalonSRX slaveLeftMotor = new WPI_TalonSRX(LEFT_MOTOR2);

  private final DifferentialDrive mDrive = new DifferentialDrive(masterLeftMotor, masterRightMotor);

  private final WPI_PigeonIMU mGyro = new WPI_PigeonIMU(GYRO_NUM);

  private final Encoder leftEncoder = new Encoder(LEFT_ENCODER_IDS[0], LEFT_ENCODER_IDS[1]);
  private final Encoder rightEncoder = new Encoder(RIGHT_ENCODER_IDS[0], RIGHT_ENCODER_IDS[1]);

  // private final DifferentialDrivePoseEstimator estimator;

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(mGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));



private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                masterLeftMotor.setVoltage(volts.in(Volts));
                masterRightMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            masterLeftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(leftEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(leftEncoder.getRate(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            masterRightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(rightEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(rightEncoder.getRate(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {


    zeroHeading();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    masterRightMotor.setInverted(true);
    slaveRightMotor.setInverted(true);

    //// Sets the distance per pulse for the encoders
    leftEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
    rightEncoder.setReverseDirection(true);

    // estimator = new DifferentialDrivePoseEstimator(DRIVE_KINEMATICS, new Rotation2d(), 0, 0,new Pose2d(2.8,5,Rotation2d.fromDegrees(180)),
    //   new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.012, 0.012, 0.012), // Local measurement standard deviations. Left encoder, right encoder, gyro.
    //   new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.015, 0.015, 0.015)); // Global measurement standard deviations. X, Y, and theta.
    field = new Field2d();


    SmartDashboard.putData("Field", field);

    SmartDashboard.putNumber("kp", 0);
    SmartDashboard.putNumber("ki", 0);
    SmartDashboard.putNumber("kd", 0);

    slaveLeftMotor.follow(masterLeftMotor);
    slaveRightMotor.follow(masterRightMotor);

    masterLeftMotor.setNeutralMode(NeutralMode.Coast);
    masterRightMotor.setNeutralMode(NeutralMode.Coast);
    slaveLeftMotor.setNeutralMode(NeutralMode.Coast);
    slaveRightMotor.setNeutralMode(NeutralMode.Coast);
    
    mGyro.reset();
    resetEncoder();

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block

    // if (NetworkTableInstance.getDefault().getTable("limelight-pom").getEntry("tv").getDouble(0) == 1) {
    //   try {

    //     if (DriverStation.getAlliance() == Alliance.Red) {
    //       botPose = NetworkTableInstance.getDefault().getTable("limelight-pom").getEntry("botpose_wpired")
    //           .getDoubleArray(new double[6]);
    //     } else {
    //       botPose = NetworkTableInstance.getDefault().getTable("limelight-pom").getEntry("botpose_wpiblue")
    //           .getDoubleArray(new double[6]);
    //     }
    //   } catch (Exception e) {
    //     botPose = NetworkTableInstance.getDefault().getTable("limelight-pom").getEntry("botpose_wpired")
    //         .getDoubleArray(new double[6]);
    //   } finally {
    //     // resetOdometry(new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(mGyro.getFusedHeading())));
    //     estimator.addVisionMeasurement(new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5])), Timer.getFPGATimestamp());
    //     // gyroOffset = botPose[5] - mGyro.getFusedHeading()


    //   }
    //   SmartDashboard.putBoolean("is target", true);
    // } 
    // else{
    //   SmartDashboard.putBoolean("is target", false);
    // }
    //   // off of prev pos seen by limelight
      
    //     estimator.update(
    //       mGyro.getRotation2d(), 
    //       masterLeftMotor.getEncoder().getPosition(),
    //       masterRightMotor.getEncoder().getPosition()
    //     );
    


    // SmartDashboard.putNumber("odometry x", getPose().getX());
    // SmartDashboard.putNumber("odometry y", getPose().getY());
    // SmartDashboard.putNumber("odometry deg", getPose().getRotation().getDegrees());
    
    
    // SmartDashboard.putNumber("right output", masterRightMotor.getAppliedOutput());
    // SmartDashboard.putNumber("left output", masterLeftMotor.getAppliedOutput());
    // SmartDashboard.putNumber("pitch", getPitchAngle());
    // SmartDashboard.putNumber("roll", mGyro.getRoll());
    // SmartDashboard.putNumber("yaw", mGyro.getYaw());
    
    odometry.update(mGyro.getRotation2d(), new DifferentialDriveWheelPositions(leftEncoder.getDistance(), rightEncoder.getDistance()));
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putNumber("left encoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("right encoder", rightEncoder.getDistance());
  }

    /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  /**
   * sets the mottors to go a distance from the curent position.
   * @param targetPosition the distance to drive
   */
  @Override
  public void setSetPoint(double target) {
    // leftPid.setReference(target - getLeftEncoder().getPosition(), CANSparkMax.ControlType.kPosition);
    // rightPid.setReference(target - getLeftEncoder().getPosition(), CANSparkMax.ControlType.kPosition);
  }

  /** returns the current pitch of the robot from gyro
   * @return the pitch angle
  */
  public double getPitchAngle() {
    return mGyro.getPitch();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**sets all of the motors to a paramater value
   * @param speed the power to set the motors to
  */
  @Override
  public void setMotor(double speed) {
    masterLeftMotor.set(speed);
    masterRightMotor.set(speed);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate() );
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    if (Math.abs(output) < Math.abs(fwd / 2)) {
      output = fwd / 2;
    }
    if (fwd - output > RATE) {
      output += RATE;
    } else if (output - fwd > RATE) {
      output -= RATE;
    }

    mDrive.arcadeDrive(output, rot);
  }
  public void simpleArcadeDrive(double fwd, double rot){
    mDrive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    masterLeftMotor.setVoltage(leftVolts);
    masterRightMotor.setVoltage(rightVolts);
    mDrive.feed();
  }

  public Command tankDriveVoltsCommand(Supplier<Double> left, Supplier<Double> right)
  {
    return new RunCommand(() -> tankDriveVolts(left.get(), right.get()), this);
  }
  /** Resets the drive encoders to currently read a position of 0. */
  @Override
  public void resetEncoder() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    mDrive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    mGyro.setFusedHeading(0);

  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return mGyro.getFusedHeading();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  // return -mGyro;
  // }

  // public void resetOdometry(Pose2d pose) {
  //   resetEncoders();
  //   mOdometry.resetPosition(
  //       new Rotation2d(mGyro.getFusedHeading() + gyroOffset), masterLeftMotor.getEncoder().getPosition(),
  //       masterRightMotor.getEncoder().getPosition(), pose);
  //   zeroHeading();
  // }

  /** finds the nearest pose to put a the picked object
   * @param autoMode the mode you want to put in
   * @return the pose to go to
   */
  // public Pose2d getPoseTarget(AutoMode autoMode) {
  //   double closestY = 0;
  //   if (autoMode == AutoMode.kCone) {
  //     for (int i = 0; i < 6; i++) {
  //       if (Math.abs(estimator.getEstimatedPosition().getY() - CONE_POSES[i]) < Math
  //           .abs(estimator.getEstimatedPosition().getY() - closestY)) {
  //         closestY = CONE_POSES[i];
  //       }
  //     }
  //     return new Pose2d(X_CONE, closestY, Rotation2d.fromDegrees(90));
  //   }
  //   if (autoMode == AutoMode.kCubeUp) {
  //     for (int i = 0; i < 3; i++) {
  //       if (Math.abs(estimator.getEstimatedPosition().getY() - CUBE_UP_POSES[i]) < Math
  //           .abs(estimator.getEstimatedPosition().getY() - closestY)) {
  //         closestY = CUBE_UP_POSES[i];
  //       }
  //     }
  //     return new Pose2d(X_CUBE_UP, closestY, Rotation2d.fromDegrees(90));

  //   }
  //   if (autoMode == AutoMode.kCubeMid) {
  //     for (int i = 0; i < 3; i++) {
  //       if (Math.abs(estimator.getEstimatedPosition().getY() - CUBE_MID_POSES[i]) < Math
  //           .abs(estimator.getEstimatedPosition().getY() - closestY)) {
  //         closestY = CUBE_MID_POSES[i];
  //       }
  //     }
  //     return new Pose2d(X_CUBE_MID, closestY, Rotation2d.fromDegrees(90));

  //   }
  //   if (autoMode == AutoMode.kHybrid) {
  //     for (int i = 0; i < 9; i++) {
  //       if (Math.abs(estimator.getEstimatedPosition().getY() - HYBRID_POSES[i]) < Math
  //           .abs(estimator.getEstimatedPosition().getY() - closestY)) {
  //         closestY = HYBRID_POSES[i];
  //       }
  //     }
  //     return new Pose2d(X_HYBRID, closestY, Rotation2d.fromDegrees(90));
  //   }
  //   return new Pose2d();
  // }

}
