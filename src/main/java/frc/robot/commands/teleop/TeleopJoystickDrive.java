// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* To do list
Be able to drive with the joystick in the teleop period
*/

public class TeleopJoystickDrive extends CommandBase {
    /** Creates a new TeleopJoystickDrive. */
    public Drivetrain drivetrain;

    private Joystick driveStick;
    private Joystick auxStick;
    private boolean fieldRelative;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param joystick  The control input for driving
     */
    public TeleopJoystickDrive(Drivetrain _subsystem, Joystick _driveStick, Joystick _auxstick,
            boolean _fieldRelative) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = _subsystem;
        this.driveStick = _driveStick;
        this.auxStick = _auxstick;
        this.fieldRelative = _fieldRelative;

        addRequirements(_subsystem);
    }

    public void SetFieldRelative(boolean setboolfieldRelative) {
        System.out.println("SetFieldRelative = " + setboolfieldRelative);
        this.fieldRelative = setboolfieldRelative;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.out.println(" JoystickDrive Running");

        // if (driveStick.getRawButtonPressed(12)) {
        // driveTrain.resetADIS16470();
        // System.out.println("m_drive.imu.reset();");
        // }

        // if (driveStick.getRawButtonPressed(10)) {
        // System.out.println("m_drive.m_odometry.resetPosition");
        // driveTrain.resetOdometry();
        // // driveTrain.m_odometry.resetPosition( new Pose2d(), new Rotation2d(0) );
        // }

        double xSpeed = this.smartJoystick(driveStick.getY() * -1, Constants.ControllerConstants.DEADZONE_DRIVE)
                * Constants.DriveConstants.MAX_DRIVE_SPEED;

        double ySpeed = this.smartJoystick(driveStick.getX() * -1, Constants.ControllerConstants.DEADZONE_DRIVE)
                * Constants.DriveConstants.MAX_DRIVE_SPEED;

        double rotRate = this.smartJoystick(driveStick.getTwist() * -1, Constants.ControllerConstants.DEADZONE_STEER)
                * Constants.DriveConstants.MAX_TWIST_RATE;

        double throttle = (-driveStick.getThrottle() + 1) / 2; // between 0 and 1 = 0% and 100%

        xSpeed *= throttle;
        ySpeed *= throttle;
        rotRate *= throttle;
        // rotRate *= 0;

        SmartDashboard.putNumber("Throttle teleJoy", throttle);

        // SmartDashboard.putNumber("teleJoy xSpeed", driveStick.getY());
        // SmartDashboard.putNumber("teleJoy ySpeed", driveStick.getX());
        // SmartDashboard.putNumber("teleJoy rotRate", driveStick.getTwist());

        SmartDashboard.putNumber("xSpeed teleJoy smart", xSpeed);
        SmartDashboard.putNumber("ySpeed teleJoy smart ", ySpeed);
        SmartDashboard.putNumber("rotRate teleJoy smart ", rotRate);

        drivetrain.drive(xSpeed, ySpeed, rotRate, fieldRelative);

    }

    /**
     * 
     * @param _in
     * @param deadZoneSize between -1 and 1
     * @return
     */
    public double smartJoystick(double _in, double deadZoneSize) {
        if (Math.abs(_in) < deadZoneSize) {
            return 0;
        }

        if (_in > 0) {
            return (_in - deadZoneSize) / (1 - deadZoneSize);
        } else if (_in < 0) {
            return (_in + deadZoneSize) / (1 - deadZoneSize);
        }
        return 0;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public void setFieldRelative(boolean bool) {
        this.fieldRelative = bool;
    }
}
