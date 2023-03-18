// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  Arm arm;
  Joystick joystick;
  double firstAngle;
  double secondAngle;
  boolean usingAngle;
  public MoveArm(Arm _arm, Joystick _joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = _arm;
    joystick = _joystick;
    usingAngle = false;
  }

  public MoveArm(Arm _arm, Joystick _joystick, double fangle, double sangle) {
    arm = _arm;
    joystick = _joystick;
    firstAngle = fangle;
    secondAngle = sangle;
    usingAngle = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.initializeArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      arm.moveTargetPoint(joystick.getX(), joystick.getY());
      arm.moveArm();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
