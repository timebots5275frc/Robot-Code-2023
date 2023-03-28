// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

    private final Translation2d leftFrontWheelLoc = new Translation2d(DriveConstants.LEFT_FRONT_WHEEL_X,
            DriveConstants.LEFT_FRONT_WHEEL_Y);
    private final Translation2d rightFrontWheelLoc = new Translation2d(DriveConstants.RIGHT_FRONT_WHEEL_X,
            DriveConstants.RIGHT_FRONT_WHEEL_Y);
    private final Translation2d rightRearWheelLoc = new Translation2d(DriveConstants.RIGHT_REAR_WHEEL_X,
            DriveConstants.RIGHT_REAR_WHEEL_Y);
    private final Translation2d leftRearWheelLoc = new Translation2d(DriveConstants.LEFT_REAR_WHEEL_X,
            DriveConstants.LEFT_REAR_WHEEL_Y);

    public final SwerveModule leftFrontSwerveModule = new SwerveModule(DriveConstants.LEFT_FRONT_DRIVE_MOTOR_ID,
            DriveConstants.LEFT_FRONT_STEER_MOTOR_ID, DriveConstants.LEFT_FRONT_STEER_ENCODER_ID);
    private final SwerveModule rightFrontSwerveModule = new SwerveModule(DriveConstants.RIGHT_FRONT_DRIVE_MOTOR_ID,
            DriveConstants.RIGHT_FRONT_STEER_MOTOR_ID, DriveConstants.RIGHT_FRONT_STEER_ENCODER_ID);
    private final SwerveModule rightRearSwerveModule = new SwerveModule(DriveConstants.RIGHT_REAR_DRIVE_MOTOR_ID,
            DriveConstants.RIGHT_REAR_STEER_MOTOR_ID, DriveConstants.RIGHT_REAR_STEER_ENCODER_ID);
    private final SwerveModule leftRearSwerveModule = new SwerveModule(DriveConstants.LEFT_REAR_DRIVE_MOTOR_ID,
            DriveConstants.LEFT_REAR_STEER_MOTOR_ID, DriveConstants.LEFT_REAR_STEER_ENCODER_ID);

    PigeonIMU gyroPigeonIMU = new PigeonIMU(Constants.DriveConstants.PIGEON_IMU_ID);
    Pigeon2 gyroPigeon2 = new Pigeon2(Constants.DriveConstants.PIGEON_2_ID);

    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontWheelLoc, rightFrontWheelLoc,
            rightRearWheelLoc, leftRearWheelLoc);
    
    public final SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {leftFrontSwerveModule.getPosition(), rightFrontSwerveModule.getPosition(),
            rightRearSwerveModule.getPosition(), leftFrontSwerveModule.getPosition()};

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, this.getHeading(), modulePositions);

    public Drivetrain() {
        System.out.println("DriveTrain (:");

    }

    @Override
    public void periodic() {
        this.updateOdometry();
        SmartDashboard.putNumber("getHeading", getHeading().getDegrees());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, this.getHeading())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        SmartDashboard.putNumber("odometry getX", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("odometry getY", m_odometry.getPoseMeters().getY());
        SmartDashboard.putString("odometry getRotation",
                m_odometry.getPoseMeters().getRotation().toString());

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_DRIVE_SPEED);

        leftFrontSwerveModule.setDesiredState(swerveModuleStates[0], true, "LF");
        rightFrontSwerveModule.setDesiredState(swerveModuleStates[1], true, "RF");
        rightRearSwerveModule.setDesiredState(swerveModuleStates[2], true, "RR");
        leftRearSwerveModule.setDesiredState(swerveModuleStates[3], true, "LR");
    }

    /* Returns the currently-estimated pose of the robot.*/
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /* Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(this.getHeading(), modulePositions);

    }

    /* Resets the odometry Position and Angle to 0. */
    public void resetOdometry() {
        System.out.println("resetOdometry");
        m_odometry.resetPosition(this.getHeading(), modulePositions, new Pose2d());
    }

    /* Resets the odometry to the specified pose. */
    public void resetOdometryWithPose2d(Pose2d pose) {
        System.out.println("resetOdometryWithPose2d");
        m_odometry.resetPosition(pose.getRotation(), modulePositions, pose);
    }

    public void resetPIgeonIMU() {
        gyroPigeon2.setYaw(0);
    }

    /* @return the robot's heading in degrees, from -180 to 180. */
    public Rotation2d getHeading() { return Rotation2d.fromDegrees(gyroPigeon2.getYaw()); }

    /* @param desiredStates The desired SwerveModule states. */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_DRIVE_SPEED);

        leftFrontSwerveModule.setDesiredState(desiredStates[0], true, "LF");
        rightFrontSwerveModule.setDesiredState(desiredStates[1], true, "RF");
        rightRearSwerveModule.setDesiredState(desiredStates[2], true, "RR");
        leftRearSwerveModule.setDesiredState(desiredStates[3], true, "LR");
    }

    public void alignWheels() {
        SwerveModuleState desiredStates = new SwerveModuleState(0, new Rotation2d(0));

        leftFrontSwerveModule.setDesiredState(desiredStates, true, "LF");
        rightFrontSwerveModule.setDesiredState(desiredStates, true, "RF");
        rightRearSwerveModule.setDesiredState(desiredStates, true, "RR");
        leftRearSwerveModule.setDesiredState(desiredStates, true, "LR");
    }

    public static Trajectory generateTrajectory(TrajectoryConfig config, List<Pose2d> list) {
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(list, config);
        return exampleTrajectory;
    }
}
