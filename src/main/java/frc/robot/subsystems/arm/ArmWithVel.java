// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.math2.Vector2;

public class ArmWithVel extends SubsystemBase {
  /** Creates a new ArmWithVel. */
  private CANSparkMax firstArmMotorController;
  private CANSparkMax secondArmMotorController;
  private RelativeEncoder firstArmEncoder;
  private RelativeEncoder secondArmEncoder;
  private SparkMaxPIDController firstArmPID;
  private SparkMaxPIDController secondArmPID;
  private CANCoder firstArmCANCoder;
  private CANCoder secondArmCANCoder;
  private double backLimit;
  private double frontLimit;

  private Vector2 ArmPointPosition;
  private Vector2 theoPos;
  private double firstArmAngle;
  private double secondArmAngle;

  // private double f_kP, f_kI, f_kD, f_kIz, f_kFF, f_maxOutput, f_minInput;

  // private double s_kP, s_kI, s_kD, s_kIz, s_kFF, s_maxOutput, s_minInput;

  private double f_kP, f_kI, f_kD, f_kIz, f_kFF, f_kMaxOutput, f_kMinOutput, f_maxRPM, f_smartMAXVelocity,
  f_smartMAXAcc, f_allowedErr, f_minVel;

  private double s_kP, s_kI, s_kD, s_kIz, s_kFF, s_kMaxOutput, s_kMinOutput, s_maxRPM, s_smartMAXVelocity,
  s_smartMAXAcc, s_allowedErr, s_minVel;

  

  public ArmWithVel() {
    firstArmMotorController = new CANSparkMax(Constants.ArmConstants.FIRST_ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondArmMotorController = new CANSparkMax(Constants.ArmConstants.SECOND_ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    firstArmEncoder = firstArmMotorController.getEncoder();
    secondArmEncoder = secondArmMotorController.getEncoder();
    firstArmPID = firstArmMotorController.getPIDController();
    secondArmPID = secondArmMotorController.getPIDController();
    firstArmCANCoder = new CANCoder(Constants.ArmConstants.FIRST_ARM_CANCODER_ID);
    secondArmCANCoder = new CANCoder(Constants.ArmConstants.SECOND_ARM_CANCODER_ID);
    firstArmEncoder.setPositionConversionFactor(1);
    secondArmEncoder.setPositionConversionFactor(1);

    // f_kP = Constants.ArmConstants.fv_kP;
    // f_kI = Constants.ArmConstants.fv_kI;
    // f_kD = Constants.ArmConstants.fv_kD;
    // f_kIz = Constants.ArmConstants.fv_kIz;
    // f_kFF = Constants.ArmConstants.fv_kFF;
    // f_maxOutput = Constants.ArmConstants.fv_maxOutput;
    // f_minInput = Constants.ArmConstants.fv_minOutput;

    // s_kP = Constants.ArmConstants.sv_kP;
    // s_kI = Constants.ArmConstants.sv_kI;
    // s_kD = Constants.ArmConstants.sv_kD;
    // s_kIz = Constants.ArmConstants.sv_kIz;
    // s_kFF = Constants.ArmConstants.sv_kFF;
    // s_maxOutput = Constants.ArmConstants.sv_maxOutput;
    // s_minInput = Constants.ArmConstants.sv_minOutput;

    // firstArmPID.setP(f_kP);
    // firstArmPID.setI(f_kI);
    // firstArmPID.setD(f_kD);
    // firstArmPID.setIZone(f_kIz);
    // firstArmPID.setFF(f_kFF);
    // firstArmPID.setOutputRange(f_minInput, f_maxOutput);
    
    // secondArmPID.setP(s_kP);
    // secondArmPID.setI(s_kI);
    // secondArmPID.setD(s_kD);
    // secondArmPID.setIZone(s_kIz);
    // secondArmPID.setFF(s_kFF);
    // secondArmPID.setOutputRange(s_minInput, s_maxOutput);

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
  }
  
  // public void moveFirstArm(Joystick joy) {
  //   if (firstArmPos < Constants.ArmConstants.backLimit - Constants.ArmConstants.slowRange && firstArmPos > Constants.ArmConstants.backLimit + Constants.ArmConstants.slowRange) {
  //     firstArmPID.setReference(joy.getY() * Constants.ArmConstants.velFactor, ControlType.kVelocity);
  //   } else if (firstArmPos < Constants.ArmConstants.backLimit && firstArmPos > Constants.ArmConstants.slowRange) {
  //     firstArmPID.setReference(joy.getY() * Constants.ArmConstants.velFactor * (1 / Math.abs(Constants.ArmConstants.backLimit - )), ControlType.kVelocity);
  //   } else if () {

  //   } else {
  //     setRefer
  //   }
  // }


  public void moveFirstArm(Joystick joy) {
    firstArmPID.setReference(joy.getY() * Constants.ArmConstants.velFactor, ControlType.kVelocity);
  }
  public void moveSecondArm(Joystick joy) {
    secondArmPID.setReference(-joy.getX() * Constants.ArmConstants.velFactor, ControlType.kVelocity);
  }
  public void moveArms(Joystick joy) {
    moveFirstArm(joy);
    moveSecondArm(joy);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (firstArmCANCoder.getAbsolutePosition() > -60 && firstArmCANCoder.getAbsolutePosition() <= 180) {
      firstArmAngle = firstArmCANCoder.getAbsolutePosition();
    } else {
      firstArmAngle = firstArmCANCoder.getAbsolutePosition() + 360;/*-firstAngle + ((180 + firstAngle) * 2); */
    }
    secondArmAngle = secondArmCANCoder.getAbsolutePosition();
  }
}
