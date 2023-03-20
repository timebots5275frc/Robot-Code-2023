// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.teleop.MoveArm;
import frc.robot.commands.teleop.MoveArmToPoint;
import frc.robot.commands.teleop.TeleopJoystickDrive;
import frc.robot.math2.Vector2;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {

  // Subsystems
  Drivetrain drivetrain = new Drivetrain();
  Arm arm = new Arm();

  //Joystic
  Joystick driveJoystick = new Joystick(0);
  Joystick armJoystick = new Joystick(1);

  //Teleop Commands
  TeleopJoystickDrive drive = new TeleopJoystickDrive(drivetrain, driveJoystick, null, true);
  MoveArm armWhenMove = new MoveArm(arm, armJoystick);
  MoveArmToPoint armMoveThing = new MoveArmToPoint(arm, new Vector2(30, 0));

  //Auto commands


  public RobotContainer() {
    drivetrain.setDefaultCommand(drive);
    arm.setDefaultCommand(armWhenMove);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //new JoystickButton(driveJoystick, 9).whileTrue();
    new JoystickButton(armJoystick, 9).whileTrue(armMoveThing);
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return null;
  }
}
