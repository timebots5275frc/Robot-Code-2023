// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.databind.introspect.AnnotationCollector.TwoAnnotations;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.math2.Vector2;

import java.util.List;
import java.nio.file.FileSystemAlreadyExistsException;
import java.util.ArrayList;

public class Arm extends SubsystemBase {
  private CANSparkMax firstArmController;
  private CANSparkMax secondArmController;
  private RelativeEncoder firstArmEncoder;
  private RelativeEncoder secondArmEncoder;
  private SparkMaxPIDController firstArmPID;
  private SparkMaxPIDController secondArmPID;
  private CANCoder firstArmCANCoder;
  private CANCoder secondArmCANCoder;

  private double f_kP, f_kI, f_kD, f_kIz, f_kFF, f_kMaxOutput, f_kMinOutput, f_maxRPM, f_smartMAXVelocity,
  f_smartMAXAcc, f_allowedErr, f_minVel;

  private double fg_kP, fg_kI, fg_kD, fg_kIz, fg_kFF, fg_kMaxOutput, fg_kMinOutput, fg_maxRPM, fg_smartMAXVelocity,
  fg_smartMAXAcc, fg_allowedErr;

  private double s_kP, s_kI, s_kD, s_kIz, s_kFF, s_kMaxOutput, s_kMinOutput, s_maxRPM, s_smartMAXVelocity,
  s_smartMAXAcc, s_allowedErr, s_minVel;

  private double sg_kP, sg_kI, sg_kD, sg_kIz, sg_kFF, sg_kMaxOutput, sg_kMinOutput, sg_maxRPM, sg_smartMAXVelocity,
  sg_smartMAXAcc, sg_allowedErr;

  private boolean over;
  
  public Vector2 targetPos = new Vector2(0, 0); // Change to set default pos
  private double targetFirstArmAngle;
  private double targetSecondArmAngle;
  private double actualFirstArmAngle;
  private double actualSecondArmAngle;
  private TwoJointInverseKinematics kinematics;



  private List<Vector2> moveSequence;

  public Arm() {
    firstArmController = new CANSparkMax(Constants.ArmConstants.FIRST_ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondArmController = new CANSparkMax(Constants.ArmConstants.SECOND_ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    firstArmEncoder = firstArmController.getEncoder();
    secondArmEncoder = secondArmController.getEncoder();
    firstArmPID = firstArmController.getPIDController();
    secondArmPID = secondArmController.getPIDController();
    kinematics = new TwoJointInverseKinematics(Constants.ArmConstants.ARM_FIRST_PART_LENGTH, Constants.ArmConstants.ARM_SECOND_PART_LENGTH);
    firstArmCANCoder = new CANCoder(Constants.ArmConstants.FIRST_ARM_CANCODER_ID);
    secondArmCANCoder = new CANCoder(Constants.ArmConstants.SECOND_ARM_CANCODER_ID);
    firstArmEncoder.setPositionConversionFactor(1);
    secondArmEncoder.setPositionConversionFactor(1);


    //PID Values
    f_kP = Constants.ArmConstants.f_kP;
    f_kI = Constants.ArmConstants.f_kI;
    f_kD = Constants.ArmConstants.f_kD;
    f_kIz = Constants.ArmConstants.f_kIz;
    f_kFF = Constants.ArmConstants.f_kFF;
    f_kMaxOutput = Constants.ArmConstants.f_kMaxOutput;
    f_kMinOutput = Constants.ArmConstants.f_kMinOutput;
    f_maxRPM = Constants.ArmConstants.f_maxRPM;
    f_smartMAXVelocity = Constants.ArmConstants.f_smartMAXVelocity;
    f_smartMAXAcc = Constants.ArmConstants.f_smartMAXAcc;
    f_allowedErr = Constants.ArmConstants.f_allowedErr;
    f_minVel = Constants.ArmConstants.f_minVelocity;

    fg_kP = Constants.ArmConstants.fg_kP;
    fg_kI = Constants.ArmConstants.fg_kI;
    fg_kD = Constants.ArmConstants.fg_kD;
    fg_kIz = Constants.ArmConstants.fg_kIz;
    fg_kFF = Constants.ArmConstants.fg_kFF;
    fg_kMaxOutput = Constants.ArmConstants.fg_kMaxOutput;
    fg_kMinOutput = Constants.ArmConstants.fg_kMinOutput;
    fg_maxRPM = Constants.ArmConstants.fg_maxRPM;
    fg_smartMAXVelocity = Constants.ArmConstants.fg_smartMAXVelocity;
    fg_smartMAXAcc = Constants.ArmConstants.fg_smartMAXAcc;
    fg_allowedErr = Constants.ArmConstants.fg_allowedErr;

    s_kP = Constants.ArmConstants.s_kP;
    s_kI = Constants.ArmConstants.s_kI;
    s_kD = Constants.ArmConstants.s_kD;
    s_kIz = Constants.ArmConstants.s_kIz;
    s_kFF = Constants.ArmConstants.s_kFF;
    s_kMaxOutput = Constants.ArmConstants.s_kMaxOutput;
    s_kMinOutput = Constants.ArmConstants.s_kMinOutput;
    s_maxRPM = Constants.ArmConstants.s_maxRPM;
    s_smartMAXVelocity = Constants.ArmConstants.s_smartMAXVelocity;
    s_smartMAXAcc = Constants.ArmConstants.s_smartMAXAcc;
    s_allowedErr = Constants.ArmConstants.s_allowedErr;
    s_minVel = Constants.ArmConstants.s_minVelocity;

    sg_kP = Constants.ArmConstants.sg_kP;
    sg_kI = Constants.ArmConstants.sg_kI;
    sg_kD = Constants.ArmConstants.sg_kD;
    sg_kIz = Constants.ArmConstants.sg_kIz;
    sg_kFF = Constants.ArmConstants.sg_kFF;
    sg_kMaxOutput = Constants.ArmConstants.sg_kMaxOutput;
    sg_kMinOutput = Constants.ArmConstants.sg_kMinOutput;
    sg_maxRPM = Constants.ArmConstants.sg_maxRPM;
    sg_smartMAXVelocity = Constants.ArmConstants.sg_smartMAXVelocity;
    sg_smartMAXAcc = Constants.ArmConstants.sg_smartMAXAcc;
    sg_allowedErr = Constants.ArmConstants.sg_allowedErr;

    firstArmPID.setP(f_kP, 0);
    firstArmPID.setI(f_kI, 0);
    firstArmPID.setD(f_kD, 0);
    firstArmPID.setIZone(f_kIz, 0);
    firstArmPID.setFF(f_kFF, 0);
    firstArmPID.setOutputRange(f_kMinOutput, f_kMaxOutput, 0);
    firstArmPID.setSmartMotionMaxVelocity(f_smartMAXVelocity, 0);
    firstArmPID.setSmartMotionMaxAccel(f_smartMAXAcc, 0);
    firstArmPID.setSmartMotionAllowedClosedLoopError(f_allowedErr, 0);
    firstArmPID.setSmartMotionMinOutputVelocity(f_minVel, 0);

    firstArmPID.setP(fg_kP, 1);
    firstArmPID.setI(fg_kI, 1);
    firstArmPID.setD(fg_kD, 1);
    firstArmPID.setIZone(fg_kIz, 1);
    firstArmPID.setFF(fg_kFF, 1);
    firstArmPID.setOutputRange(fg_kMinOutput, fg_kMaxOutput, 1);
    firstArmPID.setSmartMotionMaxVelocity(fg_smartMAXVelocity, 1);
    firstArmPID.setSmartMotionMaxAccel(fg_smartMAXAcc, 1);
    firstArmPID.setSmartMotionAllowedClosedLoopError(fg_allowedErr, 1);

    secondArmPID.setP(s_kP, 0);
    secondArmPID.setI(s_kI, 0);
    secondArmPID.setD(s_kD, 0);
    secondArmPID.setIZone(s_kIz, 0);
    secondArmPID.setFF(s_kFF, 0);
    secondArmPID.setOutputRange(s_kMinOutput, s_kMaxOutput, 0);
    secondArmPID.setSmartMotionMaxVelocity(s_smartMAXVelocity, 0);
    secondArmPID.setSmartMotionMaxAccel(s_smartMAXAcc, 0);
    secondArmPID.setSmartMotionAllowedClosedLoopError(s_allowedErr, 0);
    secondArmPID.setSmartMotionMinOutputVelocity(s_minVel, 0);

    secondArmPID.setP(sg_kP, 1);
    secondArmPID.setI(sg_kI, 1);
    secondArmPID.setD(sg_kD, 1);
    secondArmPID.setIZone(sg_kIz, 1);
    secondArmPID.setFF(sg_kFF, 1);
    secondArmPID.setOutputRange(sg_kMinOutput, sg_kMaxOutput, 1);
    secondArmPID.setSmartMotionMaxVelocity(sg_smartMAXVelocity, 1);
    secondArmPID.setSmartMotionMaxAccel(sg_smartMAXAcc, 1);
    secondArmPID.setSmartMotionAllowedClosedLoopError(sg_allowedErr, 1);

    moveSequence = new ArrayList<Vector2>();

    //Logging yippee
    over = true;
  }

  private void calculateKinematicsAngles() {
    targetFirstArmAngle = kinematics.solveFirstJoint(targetPos);
    targetSecondArmAngle = kinematics.solveSecondJoint(targetPos);
  }

  private void getActualFirstArmAngle() {
    if (firstArmCANCoder.getAbsolutePosition() >= -180 && firstArmCANCoder.getAbsolutePosition() <= -100) {
      actualFirstArmAngle = -firstArmCANCoder.getAbsolutePosition() - ((180 - firstArmCANCoder.getAbsolutePosition()) * 2);
    } else {
      actualFirstArmAngle = firstArmCANCoder.getAbsolutePosition();
    }
  }

  private void getActualSecondArmAngle() {
    actualSecondArmAngle = secondArmCANCoder.getAbsolutePosition();
  }

  public void moveTargetPoint(double joystickValue, double joystickValue2) {
    //if (!moveSequence.isEmpty()) { moveSequence.clear(); }

    targetPos.x += joystickValue * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;
    targetPos.y += -joystickValue2 * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;


    // Sus Clamping Ahhhhhh
    Vector2 normalizedVector = GetClampedPosValue(targetPos);
    targetPos = normalizedVector;
  }

  public void initializeArm() {
    targetPos = realArmPosition();
  }

  public void moveArm() {
    calculateKinematicsAngles();
    moveArm(targetFirstArmAngle, targetSecondArmAngle);
  }

  public void moveArm(double f_angle, double s_angle) {
    firstArmPID.setReference(f_angle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE, CANSparkMax.ControlType.kSmartMotion, 0);
    secondArmPID.setReference(-s_angle * Constants.ArmConstants.SECOND_ARM_ROTATIONS_PER_DEGREE, CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public void setSparkEncoders() {
    getActualFirstArmAngle();
    getActualSecondArmAngle();
    firstArmEncoder.setPosition(actualFirstArmAngle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE);
    secondArmEncoder.setPosition(-actualSecondArmAngle * Constants.ArmConstants.SECOND_ARM_ROTATIONS_PER_DEGREE);
  }

  public void changeTargetPos(Vector2 point) {
    Vector2 clampedPoint =  GetClampedPosValue(point);
    targetPos = clampedPoint;
    calculateKinematicsAngles();
    moveArm(targetFirstArmAngle, targetSecondArmAngle);
  }
  
//amonmg 
  @Override
  public void periodic() {
    if (!moveSequence.isEmpty()) { checkMoveSequence(); }
    SmartDashboard.putNumber("First Arm Mag Encoder", firstArmCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("First Arm Code Mag Encoder", (firstArmCANCoder.getAbsolutePosition() <= 180 && firstArmCANCoder.getAbsolutePosition() >= 100) ? -firstArmCANCoder.getAbsolutePosition() - ((180 - firstArmCANCoder.getAbsolutePosition()) * 2) : firstArmCANCoder.getAbsolutePosition() );
    SmartDashboard.putNumber("First Arm Spark Encoder", firstArmEncoder.getPosition());
    SmartDashboard.putNumber("Second Arm Mag Encoder", secondArmCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Second Arm Spark Encoder", secondArmEncoder.getPosition());
    SmartDashboard.putString("Target Position", targetPos.toString());
    SmartDashboard.putString("Current Position", realArmPosition().toString());
    SmartDashboard.putNumber("First Arm target rotations", targetFirstArmAngle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE);
    SmartDashboard.putNumber("Second Arm target rotations", -targetSecondArmAngle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE);
    SmartDashboard.putNumber("First Arm target degrees", targetFirstArmAngle);
    SmartDashboard.putNumber("Second Arm target degrees", targetSecondArmAngle);
    SmartDashboard.putNumber("First Arm output current", firstArmController.getOutputCurrent());
    SmartDashboard.putNumber("Second Arm output current", secondArmController.getOutputCurrent());
    SmartDashboard.putNumber("First Arm Current Rotations", firstArmEncoder.getPosition());
    SmartDashboard.putNumber("Second Arm Current Rotations", secondArmEncoder.getPosition());

  }

  public Vector2 GetClampedPosValue(Vector2 pos)
  {
    Vector2 out = pos;
    if (pos.magnitude() < Math.abs(ArmConstants.ARM_FIRST_PART_LENGTH - ArmConstants.ARM_SECOND_PART_LENGTH) + .5) { out = pos.normalized().times(Math.abs(ArmConstants.ARM_FIRST_PART_LENGTH - ArmConstants.ARM_SECOND_PART_LENGTH) + .5);}
    out = Vector2.clampMagnitude(out, ArmConstants.ARM_FIRST_PART_LENGTH + ArmConstants.ARM_SECOND_PART_LENGTH - .5);
    return out;

    /*Vector2 clampedPos = new Vector2(pos.x, pos.y);
    
    Vector2 betweenIndexs = GetConstraintsBetween(pos); // x is first index, y is second index
    Vector2 diffBetweenPoints = ArmConstants.ground_Constraints[(int)betweenIndexs.x].substract(ArmConstants.ground_Constraints[(int)betweenIndexs.y]);
    double percentBetweenIndexs = PercentBetweenNumbers(pos.x, ArmConstants.ground_Constraints[(int)betweenIndexs.x].x, ArmConstants.ground_Constraints[(int)betweenIndexs.y].x);
    Vector2 clampPos = Vector2.lerp(ArmConstants.ground_Constraints[(int)betweenIndexs.x], ArmConstants.ground_Constraints[(int)betweenIndexs.y], percentBetweenIndexs); // Position at the percent (lerped)
    
    if (Vector2.distance(pos, clampPos) <= ArmConstants.gripperRadius || clampPos.y > pos.y + ArmConstants.gripperRadius) { clampedPos = clampPos.add(Vector2.clampMagnitude(new Vector2(diffBetweenPoints.y > 0 ? -1 : 1, 1), 1).times(ArmConstants.gripperRadius)); }
    return Vector2.clampMagnitude(clampedPos, kinematics.totalDistance()); // Clamps  value to total distance of the arm
  }

  Vector2 GetConstraintsBetween(Vector2 pos) {
    for (int i = 0; i < ArmConstants.ground_Constraints.length; i++) {
      if (ArmConstants.ground_Constraints[i].x < pos.x && i + 1 < ArmConstants.ground_Constraints.length && ArmConstants.ground_Constraints[i + 1].x > pos.x) {
        return new Vector2(i, i + 1);
      }
    }

    return new Vector2(-1, -1);*/
  }

  public double PercentBetweenNumbers(double value, double min, double max) {
    double offset = 0 - min;
    return (value + offset) / (max + offset);
  }

  public void goToPoint(Vector2 pointToGoTo) // for creating move sequence
  {
    Vector2 armPos = realArmPosition();
    double inchesBetweenPoints = Vector2.distance(armPos, pointToGoTo);
    int totalPoints = (int)Math.ceil(inchesBetweenPoints * ArmConstants.Move_Points_Per_Inch);
    moveSequence.clear();

    for (int i = 0; i <= totalPoints; i++)
    {
      moveSequence.add(GetClampedPosValue(Vector2.lerp(armPos, pointToGoTo, i / (double)totalPoints)));
    }

    changeTargetPos(moveSequence.get(0));
  }

  private void checkMoveSequence()
  {
    Vector2 armPos = realArmPosition();

    if (Vector2.distance(armPos, moveSequence.get(0)) <= ArmConstants.Move_Sequence_Allowed_Error)
    {
      moveSequence.remove(0);

      if (!moveSequence.isEmpty())
      {
        Vector2 nextPoint = moveSequence.get(0);
        changeTargetPos(nextPoint);
      }
    }
  }

  public Vector2 realArmPosition()
  {
    getActualFirstArmAngle();
    getActualSecondArmAngle();

    Vector2 firstArmPos = Vector2.RadToVector2(actualFirstArmAngle * ArmConstants.DEG_TO_RAD_RATIO).times(ArmConstants.ARM_FIRST_PART_LENGTH);
    Vector2 secondArmPos = Vector2.RadToVector2((actualFirstArmAngle - actualSecondArmAngle) * ArmConstants.DEG_TO_RAD_RATIO).times(ArmConstants.ARM_SECOND_PART_LENGTH);
    Vector2 thisPos = firstArmPos.add(secondArmPos);
    thisPos.y *= -1;
    return thisPos;
  }
}
