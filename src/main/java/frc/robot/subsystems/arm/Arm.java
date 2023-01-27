// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.fasterxml.jackson.databind.introspect.AnnotationCollector.TwoAnnotations;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Arm extends SubsystemBase {
  private CANSparkMax firstArmController;
  private CANSparkMax secondArmController;
  private RelativeEncoder firstArmEncoder;
  private RelativeEncoder secondArmEncoder;
  private SparkMaxPIDController firstArmPID;
  private SparkMaxPIDController secondArmPID;

  private double f_kP, f_kI, f_kD, f_kIz, f_kFF, f_kMaxOutput, f_kMinOutput, f_maxRPM, f_smartMAXVelocity,
  f_smartMAXAcc, f_allowedErr;

  private double fg_kP, fg_kI, fg_kD, fg_kIz, fg_kFF, fg_kMaxOutput, fg_kMinOutput, fg_maxRPM, fg_smartMAXVelocity,
  fg_smartMAXAcc, fg_allowedErr;

  private double s_kP, s_kI, s_kD, s_kIz, s_kFF, s_kMaxOutput, s_kMinOutput, s_maxRPM, s_smartMAXVelocity,
  s_smartMAXAcc, s_allowedErr;

  private double sg_kP, sg_kI, sg_kD, sg_kIz, sg_kFF, sg_kMaxOutput, sg_kMinOutput, sg_maxRPM, sg_smartMAXVelocity,
  sg_smartMAXAcc, sg_allowedErr;

  private boolean over;
  
  private double xValue;//x
  private double yValue;//y
  private double distance;
  private double firstArmAngle;
  private double secondArmAngle;
  private TwoJointInverseKinematics kinematics;

  public Arm() {
    firstArmController = new CANSparkMax(Constants.ArmConstants.FIRST_ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondArmController = new CANSparkMax(Constants.ArmConstants.SECOND_ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    firstArmEncoder = firstArmController.getEncoder();
    secondArmEncoder = secondArmController.getEncoder();
    firstArmEncoder.setPositionConversionFactor(Constants.ArmConstants.FIRST_ARM_MOTOR_ROTATION_RATIO);
    secondArmEncoder.setPositionConversionFactor(Constants.ArmConstants.SECOND_ARM_MOTOR_ROTATION_RATIO);
    firstArmPID = firstArmController.getPIDController();
    secondArmPID = secondArmController.getPIDController();
    kinematics = new TwoJointInverseKinematics(Constants.ArmConstants.ARM_FIRST_PART_LENGTH, Constants.ArmConstants.ARM_SECOND_PART_LENGTH);

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

  private void calculate() {
    firstArmAngle = kinematics.solveFirstJoint(xValue, yValue);
    secondArmAngle = kinematics.solveSecondJoint(xValue, yValue);
  }

  public void movePoint(double joystickValue, double joystickValue2) {
    xValue += joystickValue * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;
    yValue += -joystickValue2 * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;


    // Sus Clamping Ahhhhhh
    double[] normalizedVector = kinematics.normalizeVector(xValue, yValue);
    normalizedVector[0] = normalizedVector[0] < 0 ? 0 : normalizedVector[0];
    normalizedVector[1] = normalizedVector[1] < 0 ? 0 : normalizedVector[1];

    xValue = normalizedVector[0];
    yValue = normalizedVector[1];
  }

  public void moveArm() {
    calculate();

    if (over) {
      firstArmPID.setReference(firstArmAngle * Constants.ArmConstants.FIRST_ARM_MOTOR_ROTATION_RATIO, CANSparkMax.ControlType.kSmartMotion, 0);
      secondArmPID.setReference(firstArmAngle * Constants.ArmConstants.FIRST_ARM_MOTOR_ROTATION_RATIO, CANSparkMax.ControlType.kSmartMotion, 0);
    } else {
      firstArmPID.setReference(firstArmAngle * Constants.ArmConstants.SECOND_ARM_MOTOR_ROTATION_RATIO, CANSparkMax.ControlType.kSmartMotion, 1);
      secondArmPID.setReference(firstArmAngle * Constants.ArmConstants.SECOND_ARM_MOTOR_ROTATION_RATIO, CANSparkMax.ControlType.kSmartMotion, 1);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
