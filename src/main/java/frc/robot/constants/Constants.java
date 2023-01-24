// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
     * @Title DrveConstants
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

        public static final double LEFT_FRONT_WHEEL_X = 11.75 * 0.0254; // meters
        public static final double LEFT_FRONT_WHEEL_Y = 11.75 * 0.0254; // meters .5969
        public static final double RIGHT_FRONT_WHEEL_X = 11.75 * 0.0254; // meters
        public static final double RIGHT_FRONT_WHEEL_Y = -11.75 * 0.0254; // meters
        public static final double RIGHT_REAR_WHEEL_X = -11.75 * 0.0254; // meters
        public static final double RIGHT_REAR_WHEEL_Y = -11.75 * 0.0254; // meters
        public static final double LEFT_REAR_WHEEL_X = -11.75 * 0.0254; // meters
        public static final double LEFT_REAR_WHEEL_Y = 11.75 * 0.0254; // meters

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
        public static final int FIRST_ARM_MOTOR_ID = 0;
        public static final int SECOND_ARM_MOTOR_ID = 0;
        public static final double ARM_FIRST_PART_LENGTH = 0;
        public static final double ARM_SECOND_PART_LENGTH = 0;
        public static final double POINT_MOVEMENT_FACTOR = 0;
        public static final double ROTATIONS_TO_ANGLE = 0;
        public static final double ARM_FIRST_PART_LIMIT = 0;
        public static final double ARM_SECOND_PART_LIMIT = 0;
        public static final double RAD_TO_DEG_RATIO = 57.2957795131;
    }
}
