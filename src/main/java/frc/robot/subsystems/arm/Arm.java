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

import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ArmConstants;

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
  f_smartMAXAcc, f_allowedErr;

  private double fg_kP, fg_kI, fg_kD, fg_kIz, fg_kFF, fg_kMaxOutput, fg_kMinOutput, fg_maxRPM, fg_smartMAXVelocity,
  fg_smartMAXAcc, fg_allowedErr;

  private double s_kP, s_kI, s_kD, s_kIz, s_kFF, s_kMaxOutput, s_kMinOutput, s_maxRPM, s_smartMAXVelocity,
  s_smartMAXAcc, s_allowedErr;

  private double sg_kP, sg_kI, sg_kD, sg_kIz, sg_kFF, sg_kMaxOutput, sg_kMinOutput, sg_maxRPM, sg_smartMAXVelocity,
  sg_smartMAXAcc, sg_allowedErr;

  private boolean over;
  
  public Vector2 pos = new Vector2(0, 0); // Change to set default pos
  private double distance;
  private double firstArmAngle;
  private double secondArmAngle;
  private double firstArmCurrentAngle;
  private double secondArmCurrentAngle;
  private TwoJointInverseKinematics kinematics;

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

    s_kP = Constants.ArmConstants.sg_kP;
    s_kI = Constants.ArmConstants.sg_kI;
    s_kD = Constants.ArmConstants.sg_kD;
    s_kIz = Constants.ArmConstants.sg_kIz;
    s_kFF = Constants.ArmConstants.sg_kFF;
    s_kMaxOutput = Constants.ArmConstants.sg_kMaxOutput;
    s_kMinOutput = Constants.ArmConstants.sg_kMinOutput;
    s_maxRPM = Constants.ArmConstants.sg_maxRPM;
    s_smartMAXVelocity = Constants.ArmConstants.sg_smartMAXVelocity;
    s_smartMAXAcc = Constants.ArmConstants.sg_smartMAXAcc;
    s_allowedErr = Constants.ArmConstants.sg_allowedErr;

    firstArmPID.setP(f_kP, 0);
    firstArmPID.setI(f_kI, 0);
    firstArmPID.setD(f_kD, 0);
    firstArmPID.setIZone(f_kIz, 0);
    firstArmPID.setFF(f_kFF, 0);
    firstArmPID.setOutputRange(f_kMinOutput, f_kMaxOutput, 0);
    firstArmPID.setSmartMotionMaxVelocity(f_smartMAXVelocity, 0);
    firstArmPID.setSmartMotionMaxAccel(f_smartMAXAcc, 0);
    firstArmPID.setSmartMotionAllowedClosedLoopError(f_allowedErr, 0);

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

  private void calculateArmAngles() {
    firstArmAngle = kinematics.solveFirstJoint(pos);
    secondArmAngle = kinematics.solveSecondJoint(pos);
  }

  private void getFirstArmAngle() {
    firstArmCurrentAngle = firstArmCANCoder.getAbsolutePosition();
  }

  private void getSecondArmAngle() {
    secondArmAngle = secondArmCANCoder.getAbsolutePosition();
  }

  public void movePoint(double joystickValue, double joystickValue2) {
    pos.x += joystickValue * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;
    pos.y += -joystickValue2 * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;


    // Sus Clamping Ahhhhhh
    Vector2 normalizedVector = GetClampedPosValue(pos);
    pos = normalizedVector;
  }

  public void moveArm() {
    calculateArmAngles();
    getFirstArmAngle();
    getSecondArmAngle();
    firstArmEncoder.setPosition(firstArmCurrentAngle);
    secondArmEncoder.setPosition(secondArmCurrentAngle);

    if (over) {
      firstArmPID.setReference(/*test these for potential negatives*/firstArmAngle * Constants.ArmConstants.FIRST_ARM_MOTOR_ROTATION_RATIO, CANSparkMax.ControlType.kSmartMotion, 0);
      secondArmPID.setReference(-secondArmAngle * Constants.ArmConstants.FIRST_ARM_MOTOR_ROTATION_RATIO, CANSparkMax.ControlType.kSmartMotion, 0);
    } else {
      firstArmPID.setReference(-firstArmAngle * Constants.ArmConstants.SECOND_ARM_MOTOR_ROTATION_RATIO, CANSparkMax.ControlType.kSmartMotion, 1);
      secondArmPID.setReference(-firstArmAngle * Constants.ArmConstants.SECOND_ARM_MOTOR_ROTATION_RATIO, CANSparkMax.ControlType.kSmartMotion, 1);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Vector2 GetClampedPosValue(Vector2 pos)
  {
    Vector2 clampedPos = new Vector2(pos.x, pos.y);
    
    Vector2 betweenIndexs = GetConstraintsBetween(pos); // x is first index, y is second index
    double percentBetweenIndexs = PercentBetweenNumbers(pos.x, ArmConstants.ground_Constraints[(int)betweenIndexs.x].x, ArmConstants.ground_Constraints[(int)betweenIndexs.y].x);
    Vector2 clampPos = Vector2.lerp(ArmConstants.ground_Constraints[(int)betweenIndexs.x], ArmConstants.ground_Constraints[(int)betweenIndexs.y], percentBetweenIndexs); // Position at the percent (lerped)
    
    if (pos.y < clampPos.y + ArmConstants.gripperRadius) {clampedPos.y = clampPos.y + ArmConstants.gripperRadius; } // Clamps position above or equal to the farthest down position allowed
    return Vector2.clampMagnitude(clampedPos, kinematics.totalDistance()); // Clamps  value to total distance of the arm
  }

  Vector2 GetConstraintsBetween(Vector2 pos) {
    for (int i = 0; i < ArmConstants.ground_Constraints.length; i++) {
      if (ArmConstants.ground_Constraints[i].x < pos.x && i + 1 < ArmConstants.ground_Constraints.length && ArmConstants.ground_Constraints[i + 1].x > pos.x) {
        return new Vector2(i, i + 1);
      }
    }

    return new Vector2(-1, -1);
  }

  public double PercentBetweenNumbers(double value, double min, double max) {
    double offset = 0 - min;
    return (value + offset) / (max + offset);
  }
}
