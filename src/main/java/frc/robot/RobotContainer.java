// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMax;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Arm arm;

  private final Relay fan = new Relay(0);
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOSparkMax());
        arm = new Arm(new ArmIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim());
        arm = new Arm(new ArmIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {});
        arm = new Arm(new ArmIO() {});
        break;
    }

    arm.setDefaultCommand(Commands.run(() -> arm.setPosition(0), arm));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  // EX: command.a()
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        Commands.run(
            () -> drive.driveArcade(-controller.getLeftY(), -controller.getRightX()),
            drive)); // changed left to right
    controller.a().whileTrue(arm.setPositionCommand(() -> 90));
    controller.b().whileTrue(arm.setPositionCommand(() -> 180));
    controller.rightBumper().whileTrue(Commands.run(() -> fan.set(Value.kOn)));
    controller.rightBumper().whileFalse(Commands.run(() -> fan.set(Value.kOff)));

    // sysId controls
    controller
        .leftBumper()
        .whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller
        .leftTrigger()
        .whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    controller
        .x()
        .whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller
        .y()
        .whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}
