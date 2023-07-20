// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.math2.Vector2;
import java.util.ArrayList;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  Arm arm;
  Joystick joystick;
  double firstAngle;
  double secondAngle;
  boolean usingAngle;
  ArrayList<Vector2> parkingPos;
  ArrayList<Vector2> restingPos;
  ArrayList<Vector2> groundPickup;
  ArrayList<Vector2> stationPickup;
  ArrayList<Vector2> groundDrop;
  ArrayList<Vector2> secondDrop;
  ArrayList<Vector2> thirdDrop;

  public MoveArm(Arm _arm, Joystick _joystick, ArrayList<Vector2> parkingPos, ArrayList<Vector2> restingPos, ArrayList<Vector2> groundPickup, ArrayList<Vector2> stationPickup, ArrayList<Vector2> groundDrop, ArrayList<Vector2> secondDrop, ArrayList<Vector2> thirdDrop) {
    arm = _arm;
    joystick = _joystick;
    this.addRequirements(arm);
    this.parkingPos = parkingPos;
    this.restingPos = restingPos;
    this.groundPickup = groundPickup;
    this.stationPickup = stationPickup;
    this.groundDrop = groundDrop;
    this.secondDrop = secondDrop;
    this.thirdDrop = thirdDrop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.initializeArm();
    arm.setSparkEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      arm.moveArm();
      arm.moveTargetWithJoystick(secondAngle, firstAngle);
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
