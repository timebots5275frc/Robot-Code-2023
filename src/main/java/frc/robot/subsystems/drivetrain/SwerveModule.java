package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.*;
import frc.robot.constants.Constants.DriveConstants;

public class SwerveModule {

    public CANSparkMax driveMotor;
    private CANSparkMax steerMotor;
    private RelativeEncoder driveNEOMotorEncoder; // NEO build-in Encoder

    private CANCoder steerAngleEncoder;

    private PIDController steerAnglePID;
    private SparkMaxPIDController steerMotorVelocityPID;
    private SparkMaxPIDController driveMotorVelocityPID;

    public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderId) {

        driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        steerAngleEncoder = new CANCoder(steerEncoderId);

        driveNEOMotorEncoder = driveMotor.getEncoder();
        driveNEOMotorEncoder.setPositionConversionFactor(DriveConstants.DRIVE_GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE);

        /// PID Controllers ///

        steerAnglePID = new PIDController(DriveConstants.PID_Encoder_Steer.P, DriveConstants.PID_Encoder_Steer.I,
                DriveConstants.PID_Encoder_Steer.D);
        steerAnglePID.enableContinuousInput(-180, 180);

        // Get the motor controller PIDs
        steerMotorVelocityPID = steerMotor.getPIDController();
        driveMotorVelocityPID = driveMotor.getPIDController();

        // set PID coefficients
        steerMotorVelocityPID.setP(DriveConstants.PID_SparkMax_Steer.P);
        steerMotorVelocityPID.setI(DriveConstants.PID_SparkMax_Steer.I);
        steerMotorVelocityPID.setD(DriveConstants.PID_SparkMax_Steer.D);
        steerMotorVelocityPID.setIZone(DriveConstants.PID_SparkMax_Steer.Iz);
        steerMotorVelocityPID.setFF(DriveConstants.PID_SparkMax_Steer.kFF);
        steerMotorVelocityPID.setOutputRange(-1, 1);
        // set PID coefficients
        driveMotorVelocityPID.setP(DriveConstants.PID_SparkMax_Drive.P);
        driveMotorVelocityPID.setI(DriveConstants.PID_SparkMax_Drive.I);
        driveMotorVelocityPID.setD(DriveConstants.PID_SparkMax_Drive.D);
        driveMotorVelocityPID.setIZone(DriveConstants.PID_SparkMax_Drive.Iz);
        driveMotorVelocityPID.setFF(DriveConstants.PID_SparkMax_Drive.kFF);
        driveMotorVelocityPID.setOutputRange(-1, 1);

    }

    /**
     * Returns the current state of the module.
     * 
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double driveSpeed = speedFromDriveRpm(driveNEOMotorEncoder.getVelocity());
        double steerAngleRadians = Math.toRadians(steerAngleEncoder.getAbsolutePosition());

        return new SwerveModuleState(driveSpeed, new Rotation2d(steerAngleRadians));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean logit, String name) {

        double curSteerAngleRadians = Math.toRadians(steerAngleEncoder.getAbsolutePosition());

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(curSteerAngleRadians));

        // The output of the steerAnglePID becomes the steer motor rpm reference.
        double steerMotorRpm = steerAnglePID.calculate(steerAngleEncoder.getAbsolutePosition(),
                state.angle.getDegrees());

        if (logit) {
            SmartDashboard.putNumber(name + " Drive OutputCurrent", driveMotor.getOutputCurrent());
            // SmartDashboard.putNumber("Drive OutputCurrent", driveMotor.current());

            SmartDashboard.putNumber(name + " steerAngleEncoder.getAbsolutePosition()",
                    steerAngleEncoder.getAbsolutePosition());
            SmartDashboard.putNumber(name + " SteerMotorRpmCommand", steerMotorRpm);
            SmartDashboard.putNumber(name + " SteerPIDdegrees", state.angle.getDegrees());
        }

        steerMotorVelocityPID.setReference(steerMotorRpm, CANSparkMax.ControlType.kVelocity);

        double driveMotorRpm = driveRpmFromSpeed(state.speedMetersPerSecond);

        if (logit) {
            double driveSpeed = driveNEOMotorEncoder.getVelocity();
            SmartDashboard.putNumber(name + " DriveSpeedMetersPerSecond", state.speedMetersPerSecond);
            SmartDashboard.putNumber(name + " DriveMotorRpmCommand", driveMotorRpm);
            SmartDashboard.putNumber(name + " DriveMotorSpeed", driveSpeed);
        }

        driveMotorVelocityPID.setReference(driveMotorRpm, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Returns the required motor rpm from the desired wheel speed in meters/second
     * 
     * @param speedMetersPerSecond
     * @return rpm of the motor
     */
    public double driveRpmFromSpeed(double speedMetersPerSecond) {
        var rpm = speedMetersPerSecond * 60.0 / DriveConstants.WHEEL_CIRCUMFERENCE / DriveConstants.DRIVE_GEAR_RATIO;
        return -1 * rpm; // Rotation reversed due to gears.
    }

    /**
     * Returns the wheel speed in meters/second calculated from the drive motor rpm.
     * 
     * @param rpm
     * @return wheelSpeed
     */
    public double speedFromDriveRpm(double rpm) {
        var speedMetersPerSecond = rpm * DriveConstants.DRIVE_GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE / 60.0;
        return -1 * speedMetersPerSecond; // Rotation reversed due to gears.
    }

    public SwerveModulePosition getPosition() {
        double distance = driveNEOMotorEncoder.getPosition();
        return new SwerveModulePosition(distance, new Rotation2d(Math.toRadians(steerAngleEncoder.getAbsolutePosition())));
    }

}
