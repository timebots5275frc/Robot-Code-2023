// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.math2.Vector2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

public class Constants {
    /**
     * 
     * @Title ControllerConstants
     */
    public static final class ControllerConstants {
        public static final int DRIVER_STICK_CHANNEL = 0;
        public static final int AUX_STICK_CHANNEL = 1;
        public static final int XBOXCONTROLLER_CHANNEL = 3;

        public static final double DEADZONE_DRIVE = 0.1;
        public static final double DEADZONE_STEER = 0.3;
    }

    /**
     * @Title DriveConstants
     */
    public static final class DriveConstants {

        // Drivetrain Motor IDs
        // There are the CANBus IDs of the SparkMax controllers
        public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
        public static final int LEFT_FRONT_STEER_MOTOR_ID = 2;
        public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
        public static final int RIGHT_FRONT_STEER_MOTOR_ID = 4;
        public static final int LEFT_REAR_DRIVE_MOTOR_ID = 5;
        public static final int LEFT_REAR_STEER_MOTOR_ID = 6;
        public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 7;
        public static final int RIGHT_REAR_STEER_MOTOR_ID = 8;

        // Drivetrain Encoder IDs
        // These are the CANBus IDs of the CTRE CANCoders
        public static final int LEFT_FRONT_STEER_ENCODER_ID = 10;
        public static final int RIGHT_FRONT_STEER_ENCODER_ID = 11;
        public static final int LEFT_REAR_STEER_ENCODER_ID = 12;
        public static final int RIGHT_REAR_STEER_ENCODER_ID = 13;

        // These constants define the location of the wheels from the center of the
        // robot.
        // These coordinates are determined by the right hand rule.
        // Index finger points in the forward X direction, Thumb points up in the
        // positive Z direction,
        // Middle finger points left in the positive Y direction.

        public static final double LEFT_FRONT_WHEEL_X = (/* inside chassis point */11 - /* wheel center offset */2.25) * 0.0254; // meters
        public static final double LEFT_FRONT_WHEEL_Y = (14 - 2.25) * 0.0254; // meters .5969
        public static final double RIGHT_FRONT_WHEEL_X = (11 - 2.25) * 0.0254; // meters
        public static final double RIGHT_FRONT_WHEEL_Y = (-14 + 2.25) * 0.0254; // meters
        public static final double RIGHT_REAR_WHEEL_X = (-11 + 2.25) * 0.0254; // meters
        public static final double RIGHT_REAR_WHEEL_Y = (-14 + 2.25) * 0.0254; // meters
        public static final double LEFT_REAR_WHEEL_X = (-11 + 2.25) * 0.0254; // meters
        public static final double LEFT_REAR_WHEEL_Y = (14 - 2.25) * 0.0254; // meters

        public static final double WHEEL_RADIUS = 2.0 * 0.0254; // meters * 0.98
        public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // meters/revolution

        public static final double MAX_DRIVE_SPEED = 3.5; // meters/second
        public static final double MAX_STEER_RATE = .5; // rotations/second of a wheel for steer.
        public static final double MAX_TWIST_RATE = .6 * 2.0 * Math.PI; // radians/second of the robot rotation.

        // Drive motor gear ratio.
        // | Driving Gear | Driven Gear |
        // First Stage | 14 | 50 |
        // Second Stage | 28 | 16 |
        // Third Stage | 15 | 60 |
        //
        // Overall Gear Ratio = 0.1225
        // One rotation of the motor gives 0.1225 rotations of the wheel.
        // 8.163 rotations of the motor gives one rotation of the wheel.
        public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0);

        // Steer motor gear ratio
        // | Driving Gear | Driven Gear |
        // First Stage | 15 | 32 |
        // Second Stage | 10 | 40 |
        //
        // Overall Gear Ration = 0.1171875
        // One rotation of the motor gives 0.1171875 rotations of the wheel.
        // 8.533 rotations of the motor gives one rotation of the wheel.
        public static final double STEER_GEAR_RATIO = (15.0 / 32) * (10 / 40);

        public static final PIDConstants PID_SparkMax_Steer = new PIDConstants(0.0001, 0, 0, 0, 0.00005);
        public static final PIDConstants PID_Encoder_Steer = new PIDConstants(20, 10, 0);

        public static final PIDConstants PID_SparkMax_Drive = new PIDConstants(0.0003, 0, 0, 0, 0.00016);
        public static final int PIGEON_IMU_ID = 9;
        public static final int PIGEON_2_ID = 9;
    }

    public static final class AutoConstants {
        // Movement
        public static final double MAX_Speed_MetersPerSecond = 1.4;
        public static final double MAX_Speed_MetersPerSecond2 = 0.6;
        public static final double MAX_Acceleration_MetersPerSecondSquared = 0.6;

        // Rotation
        public static final double MAX_AngularSpeed_RadiansPerSecond = Math.PI * 2;
        public static final double Max_AngularAcc_RadiansPerSecondSquared = Math.PI / 2;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 6;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                MAX_AngularSpeed_RadiansPerSecond, Max_AngularAcc_RadiansPerSecondSquared);
    }

    public static final class ArmConstants {
        public static final int FIRST_ARM_MOTOR_ID = 20;
        public static final int SECOND_ARM_MOTOR_ID = 21;
        public static final int FIRST_ARM_CANCODER_ID = 30;
        public static final int SECOND_ARM_CANCODER_ID = 31;
        public static final double ARM_FIRST_PART_LENGTH = 18;
        public static final double ARM_SECOND_PART_LENGTH = 34.5; /*For some reason the X is typically +1 */
        public static final double FARTHEST_EXTENSION_POINT = 48;
        public static final double POINT_MOVEMENT_FACTOR = 0.02 /* Inch per second */ * 8;
        public static final double ARM_FIRST_PART_LIMIT = 0;
        public static final double ARM_SECOND_PART_LIMIT = 0;
        public static final double RAD_TO_DEG_RATIO = 180 / Math.PI;
        public static final double DEG_TO_RAD_RATIO = Math.PI / 180;
        public static final double FIRST_ARM_MOTOR_ROTATIONS_PER_360 = (150 * 62 * 17) / (17 * 16);
        public static final double SECOND_ARM_MOTOR_ROTATIONS_PER_360 = (150 * 32) / (16);
        public static final double FIRST_ARM_ROTATIONS_PER_DEGREE = FIRST_ARM_MOTOR_ROTATIONS_PER_360 / 360.0;
        public static final double FIRST_ARM_DEGREES_PER_ROTATION = 360.0 / FIRST_ARM_MOTOR_ROTATIONS_PER_360;
        public static final double SECOND_ARM_ROTATIONS_PER_DEGREE = SECOND_ARM_MOTOR_ROTATIONS_PER_360 / 360.0;
        public static final double SECOND_ARM_DEGREES_PER_ROTATION = 360.0 / SECOND_ARM_MOTOR_ROTATIONS_PER_360;
        public static final double sg_smartMAXVelocity = 0;
        public static final double sg_smartMAXAcc = 0;

        public static final double fv_kP = 0;
        public static final double fv_kI = 0;
        public static final double fv_kD = 0;
        public static final double fv_kIz = 0;
        public static final double fv_kFF = 0;
        public static final double fv_maxOutput = 0;
        public static final double fv_minOutput = 0;

        public static final double backLimit = 45;
        public static final double frontLimit = -15;
        public static final double slowRange  = 0;
        public static final double velFactor = 75;
        
        public static final double sv_kP = 0;
        public static final double sv_kI = 0;
        public static final double sv_kD = 0;
        public static final double sv_kIz = 0;
        public static final double sv_kFF = 0;
        public static final double sv_maxOutput = 0;
        public static final double sv_minOutput = 0;



        //PID values first arm
        public static final double f_kP = 0.000;
        public static final double f_kI = 0.000 / 1000.0;
        public static final double f_kD = 0;
        public static final double f_kIz = 0;
        public static final double f_kFF = 0.02;
        public static final double f_kMaxOutput = .75;
        public static final double f_kMinOutput = -.75;
        public static final double f_maxRPM = 0;
        public static final double f_smartMAXVelocity = 720;
        public static final double f_smartMAXAcc = 720;
        public static final double f_allowedErr = 1;
        public static final double f_minVelocity = 0;

        //PID values first arm wih gravity
        public static final double fg_kP = 0;
        public static final double fg_kI = 0;
        public static final double fg_kD = 0;
        public static final double fg_kIz = 0;
        public static final double fg_kFF = 0;
        public static final double fg_kMaxOutput = 0;
        public static final double fg_kMinOutput = 0;
        public static final double fg_maxRPM = 0;
        public static final double fg_smartMAXVelocity = 0;
        public static final double fg_allowedErr = 0;
        public static final double fg_smartMAXAcc = 0;

        //PID values first arm against gravity
        public static final double s_kP = 0.000;
        public static final double s_kI = 0.000 / 2000;
        public static final double s_kD = 0;
        public static final double s_kIz = 0;
        public static final double s_kFF = 0.02;
        public static final double s_kMaxOutput = .75;
        public static final double s_kMinOutput = -.75;
        public static final double s_maxRPM = 0;
        public static final double s_smartMAXVelocity = 360;
        public static final double s_smartMAXAcc = 360;
        public static final double s_allowedErr = 1;
        public static final double s_minVelocity = 0;

        //PID values second arm with gravity
        public static final double sg_kP = 0;
        public static final double sg_kI = 0;
        public static final double sg_kD = 0;
        public static final double sg_kIz = 0;
        public static final double sg_kFF = 0;
        public static final double sg_kMaxOutput = 0;
        public static final double sg_kMinOutput = 0;
        public static final double sg_maxRPM = 0;
        public static final double sg_allowedErr = 0;
        
        //PID loop
        //l

        // Arm Ground Constraints (Relative to arm mount points in inches)
        public static final Vector2[] ground_Constraints = { 
        new Vector2(-30.0061, -25.8670), //  Back of motor
        new Vector2(-26.5601, -26.117), //  Far left top of battery mount
        new Vector2(-23.398, -16.5532), //  Left top arm mount
        new Vector2(-22.6487, -15.424), //  Left motor
        new Vector2(-21.0854, -13.8517), //  Top motor
        new Vector2(-19.1559, -15.7821), //  Right motor
        new Vector2(-17.0811, -22.5870), //  Right battery
        new Vector2(6.9939, -25.8670), //  Front of bumper
    };
        public static final double gripperRadius = 7; // inches
        public static final double Move_Sequence_Allowed_Error = 0.5;
        public static final double Move_Points_Per_Inch = 1;

        public static final double farthestBackTargetPos = -14;
        public static final double bumperFrontXPos = 7;
        public static final double frontXPos = 20;
        public static final double robotLargestY = -9;
        public static final double robotSmallestY = -19.5;
        public static final double groundSmallestY = -27;
    }
}
