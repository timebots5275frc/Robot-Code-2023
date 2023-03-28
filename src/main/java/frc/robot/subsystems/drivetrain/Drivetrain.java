// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMUConfiguration;

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

    // private final AnalogGyro m_gyro = new AnalogGyro(0);

    //public final GyroWrapperADIS16470_IMU imuADIS16470 = new GyroWrapperADIS16470_IMU();

    PigeonIMU gyroPigeonIMU = new PigeonIMU(Constants.DriveConstants.PIGEON_IMU_ID);
    Pigeon2 gyroPigeon2 = new Pigeon2(Constants.DriveConstants.PIGEON_2_ID);

    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontWheelLoc, rightFrontWheelLoc,
            rightRearWheelLoc, leftRearWheelLoc);
    
    public final SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {leftFrontSwerveModule.getPosition(), rightFrontSwerveModule.getPosition(),
            rightRearSwerveModule.getPosition(), leftFrontSwerveModule.getPosition()};

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, this.getHeading(), modulePositions);


    private double getOdometryUpdate;


    public Drivetrain() {
        System.out.println("DriveTrain (:");

    }

    @Override
    public void periodic() {
        this.updateOdometry();
        SmartDashboard.putNumber("getHeading", getHeading().getDegrees());
        SmartDashboard.putNumber("Odometry Update Degrees", getOdometryUpdate);
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
        System.out.println("Driving at " + xSpeed + " " + ySpeed);

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, this.getHeading())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        SmartDashboard.putNumber("odometry getX", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("odometry getY", m_odometry.getPoseMeters().getY());
        SmartDashboard.putString("odometry getRotation",
                m_odometry.getPoseMeters().getRotation().toString());
        

        // SmartDashboard.putNumber("LeftFrontSpeed",
        // swerveModuleStates[0].speedMetersPerSecond );
        // SmartDashboard.putNumber("LeftFrontAngle",
        // swerveModuleStates[0].angle.getDegrees() );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_DRIVE_SPEED);
        // SmartDashboard.putNumber("LeftFrontSpeedNorm",
        // swerveModuleStates[0].speedMetersPerSecond );

        leftFrontSwerveModule.setDesiredState(swerveModuleStates[0], true, "LF");
        rightFrontSwerveModule.setDesiredState(swerveModuleStates[1], true, "RF");
        rightRearSwerveModule.setDesiredState(swerveModuleStates[2], true, "RR");
        leftRearSwerveModule.setDesiredState(swerveModuleStates[3], true, "LR");
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(this.getHeading(), modulePositions);
        getOdometryUpdate = this.getHeading().getDegrees();

    }

    /**
     * Resets the odometry Position and Angle to 0.
     */
    public void resetOdometry() {
        System.out.println("resetOdometry");
        m_odometry.resetPosition(this.getHeading(), modulePositions, new Pose2d());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometryWithPose2d(Pose2d pose) {
        System.out.println("resetOdometryWithPose2d");
        m_odometry.resetPosition(pose.getRotation(), modulePositions, pose); // imuADIS16470.getRotation2d()
    }

    /** Zeroes the heading of the robot. */
    // public void resetADIS16470() {
    // System.out.println("resetADIS16470");
    // imuADIS16470.reset();
    // }

    /** calibrate the heading of the robot. */
    // public void calibrateADIS16470() {
    // System.out.println("calibrateADIS16470");
    // imuADIS16470.calibrate();
    // }

    public void calibratePigeonIMU() {

    }

    public void resetPIgeonIMU() {
        gyroPigeon2.setYaw(0);
    }

    /**
     * Returns the heading of the robot.
     * 
     * @return the robot's heading in degrees, from -180 to 180. // ! This comment
     *         was from last year.
     */
    public Rotation2d getHeading() {

        Rotation2d heading = Rotation2d.fromDegrees(gyroPigeon2.getYaw());


        // System.out.println( "getYComplementaryAngle = " + heading );
        // System.out.println( "getXComplementaryAngle = " +
        // imuADIS16470.getXComplementaryAngle() );

        return heading; // TODO Lucas //.minus(new Rotation2d(this.autoTurnOffsetRadians)); // radians

    }

    /**
     * Sets the swerve ModuleStates.
     * 
     * @param desiredStates The desired SwerveModule states.
     */
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

    /**
     * ! We might need this for the Auto. -Lucas
     * 
     * // TODO: Find out if we still need this. -Lucas
     * // private double autoTurnOffsetRadians = 0;
     * 
     * 
     * public void setAutoTurnOffsetRadians(double angleInRadians) { // TODO -Lucas
     * System.out.println("angleInRadians = " + angleInRadians);
     * this.autoTurnOffsetRadians = angleInRadians;
     * }
     * 
     * 
     * protected static Trajectory loadTrajectory(String trajectoryName) throws
     * IOException {
     * return TrajectoryUtil.fromPathweaverJson(
     * Filesystem.getDeployDirectory().toPath().resolve(Paths.get("output",
     * trajectoryName + ".wpilib.json")));
     * }
     * 
     * public Trajectory loadTrajectoryFromFile(String filename) {
     * try {
     * return loadTrajectory(filename);
     * } catch (IOException e) {
     * DriverStation.reportError("Failed to load auto trajectory: " + filename,
     * false);
     * return new Trajectory();
     * }
     * }
     * 
     * 
     * 
     * public static Trajectory generateTrajectory(TrajectoryConfig config, Pose2d
     * startingPose2d,
     * List<Translation2d> list) {
     * // Pose2d offset = new Pose2d(startingPose2d.getX(), startingPose2d.getY(),
     * // startingPose2d.getRotation());
     * 
     * ArrayList<Translation2d> newList = new ArrayList<Translation2d>();
     * 
     * for (int i = 0; i < list.size(); i++) {
     * // double x = (list.get(i).getX() - startingPose2d.getX()) * 30 * .0254;
     * // double y = (list.get(i).getY() - startingPose2d.getY()) * 30 * .0254;
     * 
     * double x = list.get(i).getX() * 30 * .0254;
     * double y = list.get(i).getY() * 30 * .0254;
     * newList.add(new Translation2d(x, y));
     * System.out.println("x = " + x + " y = " + y);
     * }
     * 
     * Translation2d end2d = newList.remove(newList.size() - 1);
     * 
     * Pose2d end = new Pose2d(end2d.getX(), end2d.getY(),
     * startingPose2d.getRotation());
     * 
     * return TrajectoryGenerator.generateTrajectory(new
     * Pose2d(startingPose2d.getX() * 30 * .0254,
     * startingPose2d.getY() * 30 * .0254, startingPose2d.getRotation()), list, end,
     * config);
     * 
     * // Pose2d offset = new Pose2d(startingPose2d.getX(), startingPose2d.getY(),
     * // startingPose2d.getRotation());
     * // ArrayList<Pose2d> newList = new ArrayList<Pose2d>();
     * // newList.add(list.get(0));
     * 
     * // for (int i = 1; i < list.size(); i++) {
     * // double x = (list.get(i).getX() - offset.getX()) * 30 * .0254;
     * // double y = (list.get(i).getY() - offset.getY()) * 30 * .0254;
     * // newList.add(new Pose2d(x, y, list.get(i).getRotation()));
     * // System.out.println("x = " + x + " y = " + y);
     * // }
     * 
     * // Trajectory exampleTrajeactory =
     * // TrajectoryGenerator.generateTrajectory(newList, config);
     * 
     * // return exampleTrajeactory;
     * }
     */

}
