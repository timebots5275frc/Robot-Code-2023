// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.math2.Vector2;
import frc.robot.subsystems.arm.Arm;

public class MoveArmToPoint extends CommandBase {
  /** Creates a new MoveArmToPoint. */
  Arm arm;
  Vector2 point;
  public MoveArmToPoint(Arm _arm, Vector2 _point) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = _arm;
    point = _point;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.moveArm(point);
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
