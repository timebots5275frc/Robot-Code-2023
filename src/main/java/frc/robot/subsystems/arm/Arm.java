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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  
  public Vector2 targetPos = new Vector2(0, 0); // Change to set default pos
  private double targetFirstArmAngle;
  private double targetSecondArmAngle;
  private double actualFirstArmAngle;
  private double actualSecondArmAngle;
  private double targetFirstArmVelocity;
  private double targetSecondArmVelocity;
  private double convertDPStoRPMf;
  private double convertDPStoRPMs;
  // private double loopTime;
  // private double previousTime;
  private TwoJointInverseKinematics kinematics;
  double firstArmAngleDiff;
  double secondArmAngleDiff;

  // private ArrayList<Vector2> currentMoveSequence;
  // private int moveSequenceIndex;
  private long lastPeriodicTime = 0;
  private double deltaTime;

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
    firstArmAngleDiff = 0;
    secondArmAngleDiff = 0;

    //PID Values
    f_kP = Constants.ArmConstants.f_kP;
    f_kI = Constants.ArmConstants.f_kI;
    f_kD = Constants.ArmConstants.f_kD;
    f_kIz = Constants.ArmConstants.f_kIz;
    f_kFF = Constants.ArmConstants.f_kFF;
    f_kMaxOutput = Constants.ArmConstants.f_kMaxOutput;
    f_kMinOutput = Constants.ArmConstants.f_kMinOutput;
    // f_maxRPM = Constants.ArmConstants.f_maxRPM;
    // f_smartMAXVelocity = Constants.ArmConstants.f_smartMAXVelocity;
    // f_smartMAXAcc = Constants.ArmConstants.f_smartMAXAcc;
    // f_allowedErr = Constants.ArmConstants.f_allowedErr;
    // f_minVel = Constants.ArmConstants.f_minVelocity;

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
    // s_maxRPM = Constants.ArmConstants.s_maxRPM;
    // s_smartMAXVelocity = Constants.ArmConstants.s_smartMAXVelocity;
    // s_smartMAXAcc = Constants.ArmConstants.s_smartMAXAcc;
    // s_allowedErr = Constants.ArmConstants.s_allowedErr;
    // s_minVel = Constants.ArmConstants.s_minVelocity;

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
    // firstArmPID.setSmartMotionMaxVelocity(f_smartMAXVelocity, 0);
    // firstArmPID.setSmartMotionMaxAccel(f_smartMAXAcc, 0);
    // firstArmPID.setSmartMotionAllowedClosedLoopError(f_allowedErr, 0);
    // firstArmPID.setSmartMotionMinOutputVelocity(f_minVel, 0);

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
    // secondArmPID.setSmartMotionMaxVelocity(s_smartMAXVelocity, 0);
    // secondArmPID.setSmartMotionMaxAccel(s_smartMAXAcc, 0);
    // secondArmPID.setSmartMotionAllowedClosedLoopError(s_allowedErr, 0);
    // secondArmPID.setSmartMotionMinOutputVelocity(s_minVel, 0);

    secondArmPID.setP(sg_kP, 1);
    secondArmPID.setI(sg_kI, 1);
    secondArmPID.setD(sg_kD, 1);
    secondArmPID.setIZone(sg_kIz, 1);
    secondArmPID.setFF(sg_kFF, 1);
    secondArmPID.setOutputRange(sg_kMinOutput, sg_kMaxOutput, 1);
    secondArmPID.setSmartMotionMaxVelocity(sg_smartMAXVelocity, 1);
    secondArmPID.setSmartMotionMaxAccel(sg_smartMAXAcc, 1);
    secondArmPID.setSmartMotionAllowedClosedLoopError(sg_allowedErr, 1);
  }

  private void calculateTargetVelocity()
  {
    calculateKinematicsAngles();
    getActualFirstArmAngle();
    getActualSecondArmAngle();

    firstArmAngleDiff = targetFirstArmAngle - actualFirstArmAngle;
    secondArmAngleDiff = targetSecondArmAngle - actualSecondArmAngle; 

    convertDPStoRPMf = ((firstArmAngleDiff * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE) * 60) / deltaTime;

    convertDPStoRPMs = ((secondArmAngleDiff * Constants.ArmConstants.SECOND_ARM_ROTATIONS_PER_DEGREE) / deltaTime) * 60;

  }

  private void calculateKinematicsAngles() {
    targetFirstArmAngle = kinematics.solveFirstJoint(targetPos);
    targetSecondArmAngle = kinematics.solveSecondJoint(targetPos);
  }

  private void getActualFirstArmAngle() {
    if (firstArmCANCoder.getAbsolutePosition() > -60 && firstArmCANCoder.getAbsolutePosition() <= 180) {
       actualFirstArmAngle = firstArmCANCoder.getAbsolutePosition();
    } else {
       actualFirstArmAngle = firstArmCANCoder.getAbsolutePosition() + 360;
    }
  }

  private void getActualSecondArmAngle() {
    actualSecondArmAngle = secondArmCANCoder.getAbsolutePosition();
  }

  // public void getInput(double joystickValue, double joystickValue2, Joystick joy) {
  //   checkPresetMoveSequenceButtons(joy);
  //   if (currentMoveSequence.size() == 0) { moveTargetWithJoystick(joystickValue, joystickValue2); }
  //   else {}
  // } 
  // Move Sequence Code 

  public void moveTargetWithJoystick(double joystickValue, double joystickValue2) {
    targetPos.x += joystickValue * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;
    targetPos.y += -joystickValue2 * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;
    
    // Clamping for point
    Vector2 normalizedVector = GetClampedPosValue(targetPos);
    targetPos = normalizedVector;
    calculateTargetVelocity();
  }

  // void checkPresetMoveSequenceButtons(Joystick joy)
  // {
  //   if (joy.getRawButtonPressed(1)) { setNewMoveSequence(Constants.ArmConstants.ParkingPos); } 
  //   else if (joy.getRawButtonPressed(12)) { setNewMoveSequence(Constants.ArmConstants.RestingPos); } 
  //   else if (joy.getRawButtonPressed(8)) { setNewMoveSequence(Constants.ArmConstants.GrabFromGroundPos); } 
  //   else if (joy.getRawButtonPressed(10)) { setNewMoveSequence(Constants.ArmConstants.GrabFromStationPos);  }
  //   else if (joy.getRawButtonPressed(7)) { setNewMoveSequence(Constants.ArmConstants.PlaceOnGroundPos); } 
  //   else if (joy.getRawButtonPressed(9)) { setNewMoveSequence(Constants.ArmConstants.PlaceOnSecondPos); } 
  //   else if (joy.getRawButtonPressed(11)) { setNewMoveSequence(Constants.ArmConstants.PlaceOnThirdPos); }
  // }
  // More move sequence

  // void setNewMoveSequence(ArrayList<Vector2> newSequence)
  // {
  //   Vector2 armPos = realArmPosition();
  //   currentMoveSequence = newSequence;

  //   int closestPointIndex = -1;

  //   for (int i = 0; i < currentMoveSequence.size(); i++) {
  //     double distanceFromCurrentPoint = Vector2.distance(currentMoveSequence.get(i), armPos);

  //     if (closestPointIndex == -1 || distanceFromCurrentPoint < Vector2.distance(currentMoveSequence.get(closestPointIndex), armPos))
  //     {
  //       closestPointIndex = i;
  //     }
  //   }

  //   moveSequenceIndex = closestPointIndex;
  // }
  // More Move Sequence

  // void checkMoveSequence()
  // {
  //   if (currentMoveSequence.size() > 0)
  //   {
  //     Vector2 currentTargetPoint = currentMoveSequence.get(moveSequenceIndex);
  //     Vector2 currentArmPos = realArmPosition();

  //     if (!targetPos.equals(currentTargetPoint)) { changeTargetPos(currentTargetPoint); }

  //     if (Vector2.distance(currentTargetPoint, currentArmPos) <= ArmConstants.Move_Sequence_Allowed_Error) 
  //     {
  //       if (moveSequenceIndex + 1 < currentMoveSequence.size()) { moveSequenceIndex++; }
  //       else { currentMoveSequence.clear(); moveSequenceIndex = 0; }
  //     }
  //   }
  // }
  // More Move Sequence

  public void initializeArm() {
    targetPos = realArmPosition();
  }

  // public void moveArm() {
  //   calculateKinematicsAngles();
  //   moveArm(targetFirstArmAngle, targetSecondArmAngle);
  // }

  // public void moveArm(double f_angle, double s_angle) {
  //   firstArmPID.setReference(f_angle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE, CANSparkMax.ControlType.kSmartMotion, 0);
  //   secondArmPID.setReference(-s_angle * Constants.ArmConstants.SECOND_ARM_ROTATIONS_PER_DEGREE, CANSparkMax.ControlType.kSmartMotion, 0);
  // }

  //Old Move Arm

  public void moveArm() {
    firstArmPID.setReference(convertDPStoRPMf, CANSparkMax.ControlType.kVelocity);
    secondArmPID.setReference(convertDPStoRPMs, CANSparkMax.ControlType.kVelocity);
  }

  public void setSparkEncoders() {
    getActualFirstArmAngle();
    getActualSecondArmAngle();
    firstArmEncoder.setPosition(actualFirstArmAngle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE);
    secondArmEncoder.setPosition(-actualSecondArmAngle * Constants.ArmConstants.SECOND_ARM_ROTATIONS_PER_DEGREE);
  }

  // public void changeTargetPos(Vector2 point) {
  //   Vector2 clampedPoint =  GetClampedPosValue(point);
  //   targetPos = clampedPoint;
  //   calculateKinematicsAngles();
  //   moveArm(targetFirstArmAngle, targetSecondArmAngle);
  // }
  
  @Override
  public void periodic() {
    deltaTime = (double)(System.currentTimeMillis() - lastPeriodicTime) / 1000;
    lastPeriodicTime = System.currentTimeMillis();

    Vector2 armPos = realArmPosition();
    double calculatedAngle;
    if (firstArmCANCoder.getAbsolutePosition() > 0 && armPos.x < 0) {
      calculatedAngle = firstArmCANCoder.getAbsolutePosition();
    } else {
      calculatedAngle = -firstArmCANCoder.getAbsolutePosition() + ((180 + firstArmCANCoder.getAbsolutePosition()) * 2);
    }

    double adjustedFirstArmAngle;
    if (firstArmCANCoder.getAbsolutePosition() > -60 && firstArmCANCoder.getAbsolutePosition() <= 180) {
      adjustedFirstArmAngle = firstArmCANCoder.getAbsolutePosition();
    } else {
      adjustedFirstArmAngle = firstArmCANCoder.getAbsolutePosition() + 360;
    }

    LogSmartDashboard(calculatedAngle, adjustedFirstArmAngle);

  }

  public Vector2 GetClampedPosValue(Vector2 pos)
  {
    Vector2 o = pos;
    double armDiff = Math.abs(ArmConstants.ARM_FIRST_PART_LENGTH - ArmConstants.ARM_SECOND_PART_LENGTH);
    double farthestArmReach = kinematics.totalDistance();

    if (o.x < ArmConstants.frontOfBumperXPos) // Inside chassis
    {
        if (o.y > ArmConstants.insideChassisLargestY && o.x < 0) { if (o.y >= ArmConstants.insideChassisLargestY) { o.y = realArmPosition().y; } }
        else if (o.y < ArmConstants.insideChassisSmallestY) { o.y = ArmConstants.insideChassisSmallestY; }
    }
    else // Outside chassis
    {
        double p = PercentBetweenNumbers(o.x, ArmConstants.frontOfBumperXPos, ArmConstants.frontGroundXPos);
        Vector2 point3 = new Vector2(ArmConstants.frontOfBumperXPos, ArmConstants.insideChassisSmallestY);
        Vector2 point4 = new Vector2(ArmConstants.frontGroundXPos, ArmConstants.outsideChassisSmallestY);
        o.y = clampNumber(o.y, Vector2.lerp(point3, point4, p).y, farthestArmReach);

        if (o.y < ArmConstants.outsideChassisSmallestY) { o.y = ArmConstants.outsideChassisSmallestY; }
    }

    if (o.y >= 0 && o.x < armDiff) { o.x = armDiff; } // Above y=0

    if (o.magnitude() < armDiff + .5f) { o = o.normalized().times(armDiff + .5f); } // Clamp outside min circle
    o = Vector2.clampMagnitude(o, farthestArmReach - .5f); // Clamp inside max circle
    o.x = clampNumber(o.x, ArmConstants.farthestBackChassisPos, ArmConstants.FARTHEST_EXTENSION_POINT); // Clamping between smallest and largest allowed x value

    return o;
  }

  double clampNumber(double val, double min, double max) {
    if (val < min) { return min; }
    else if (val > max) { return max; }
    return val;
  }

  public double PercentBetweenNumbers(double value, double min, double max) {
    double offset = 0 - min;
    return (value + offset) / (max + offset);
  }

  // Getting the current arms position

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

  // Logging values

  void LogSmartDashboard(double calculatedAngle, double adjustedFirstArmAngle)
  {
    SmartDashboard.putNumber("First Arm Mag Encoder", firstArmCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Calculated angle", calculatedAngle);
    SmartDashboard.putNumber("Actual First Arm Angle", adjustedFirstArmAngle);

    SmartDashboard.putNumber("Real current rots", actualFirstArmAngle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE);
    SmartDashboard.putNumber("First Arm Spark Encoder", firstArmEncoder.getPosition());
    SmartDashboard.putNumber("Second Arm Mag Encoder", secondArmCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Second Arm Spark Encoder", secondArmEncoder.getPosition());
    SmartDashboard.putString("Target Position", targetPos.toString());
    SmartDashboard.putString("Current Position", realArmPosition().toString());
    SmartDashboard.putNumber("First Arm target rotations", targetFirstArmAngle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE);
    SmartDashboard.putNumber("Second Arm target rotations", -targetSecondArmAngle * Constants.ArmConstants.SECOND_ARM_ROTATIONS_PER_DEGREE);
    SmartDashboard.putNumber("First Arm target degrees", targetFirstArmAngle);
    SmartDashboard.putNumber("Second Arm target degrees", targetSecondArmAngle);
    SmartDashboard.putNumber("First Arm output current", firstArmController.getOutputCurrent());
    SmartDashboard.putNumber("Second Arm output current", secondArmController.getOutputCurrent());
    SmartDashboard.putNumber("First Arm Current Rotations", firstArmEncoder.getPosition());
    SmartDashboard.putNumber("Second Arm Current Rotations", secondArmEncoder.getPosition());
    SmartDashboard.putNumber("First Arm Motor Speed", firstArmEncoder.getVelocity());
    SmartDashboard.putNumber("Second Arm Motor Speed", secondArmEncoder.getVelocity());
    SmartDashboard.putNumber("First Arm Target Velocity", convertDPStoRPMf);
    SmartDashboard.putNumber("Second Arm Target Velocity", convertDPStoRPMs);
    SmartDashboard.putNumber("First Arm Angle Diff", firstArmAngleDiff);
    SmartDashboard.putNumber("Second Arm Angle Diff", secondArmAngleDiff);

  }
}
