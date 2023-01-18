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
  /** Creates a new Arm. */
  private CANSparkMax firstArmController;
  private CANSparkMax secondArmController;
  private RelativeEncoder firstArmEncoder;
  private RelativeEncoder secondArmEncoder;
  private SparkMaxPIDController firstArmPID;
  private SparkMaxPIDController secondArmPID;

  Vector test;

  private double f_kP, f_kI, f_kD, f_kIz, f_kFF, f_kMaxOutput, f_kMinOutput, f_maxRPM, f_smartMAXVelocity,
  f_smartMAXAcc, f_allowedErr;

  private double s_kP, s_kI, s_kD, s_kIz, s_kFF, s_kMaxOutput, s_kMinOutput, s_maxRPM, s_smartMAXVelocity,
  s_smartMAXAcc, s_allowedErr;
  
  private double xValue;
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
    firstArmEncoder.setPositionConversionFactor(Constants.ArmConstants.ROTATIONS_TO_ANGLE);
    secondArmEncoder.setPositionConversionFactor(Constants.ArmConstants.ROTATIONS_TO_ANGLE);
    firstArmPID = firstArmController.getPIDController();
    secondArmPID = secondArmController.getPIDController();
    kinematics = new TwoJointInverseKinematics(Constants.ArmConstants.ARM_FIRST_PART_LENGTH, Constants.ArmConstants.ARM_SECOND_PART_LENGTH);
  }

  private void calculate() {
    firstArmAngle = kinematics.solveFirstJoint(xValue, yValue);
    secondArmAngle = kinematics.solveSecondJoint(xValue, yValue);
  }

  public void movePoint(double joystickValue, double joystickValue2) {
    xValue += joystickValue * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;
    yValue += joystickValue2 * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;


    // Sus Clamping
    double[] normalizedVector = kinematics.normalizeVector(xValue, yValue);
    normalizedVector[0] = normalizedVector[0] < 0 ? 0 : normalizedVector[0];
    normalizedVector[1] = normalizedVector[1] < 0 ? 0 : normalizedVector[1];

    xValue = normalizedVector[0];
    yValue = normalizedVector[1];
  }

  public void moveArm() {
    
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
