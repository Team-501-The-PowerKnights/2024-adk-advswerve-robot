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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Incrementer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Mast;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake m_intake;
  private final Feeder m_feeder;
  private final Incrementer m_incrementer;
  private final Launcher m_launcher;
  private final Mast m_mast;

  // Controller
  private final CommandXboxController driverPad = new CommandXboxController(0);
  private final CommandXboxController operPad = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(false),
                new ModuleIOSparkFlex(0), // FL
                new ModuleIOSparkFlex(1), // FR
                new ModuleIOSparkFlex(2), // BL
                new ModuleIOSparkFlex(3)); // BR
        // flywheel = new Flywheel(new FlywheelIOSparkMax());
        // drive = new Drive(
        // new GyroIOPigeon2(true),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        // flywheel = new Flywheel(new FlywheelIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // flywheel = new Flywheel(new FlywheelIO() {});
        break;
    }

    // m_intake = null;
    m_intake = new Intake();

    // m_feeder = null;
    m_feeder = new Feeder();

    // m_incrementer = null;
    m_incrementer = new Incrementer();

    m_launcher = null;
    // m_launcher = new Launcher();

    m_mast = null;
    //  m_mast = new Mast();

    // Configure the button bindings
    configureButtonBindings();
  }

  // STU
  private static class ZeroSupplier {
    public static double zero() {
      return 0.0;
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // STU
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> (-driverPad.getLeftY() * 0.85),
            () -> (-driverPad.getLeftX() * 0.85),
            () -> (driverPad.getRightX() * 0.85)));
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -controller.getLeftY(),
    //         () -> ZeroSupplier.zero(),
    //         () -> ZeroSupplier.zero()));
    // DriveCommands.joystickDrive(
    //     drive,
    //     () -> ZeroSupplier.zero(),
    //     () -> ZeroSupplier.zero(),
    //     () -> ZeroSupplier.zero()));

    // Intake controls:
    operPad.leftBumper().whileTrue(m_intake.runIntake());
    operPad.leftTrigger().whileTrue(m_intake.reverseIntake());

    // Feeder Controls:
    operPad.rightBumper().whileTrue(m_feeder.runFeeder());
    operPad.rightTrigger().whileTrue(m_feeder.reverseFeeder());

    // Incrementer Controls:
    operPad.b().whileTrue(m_incrementer.runIncrementer());
    operPad.y().whileTrue(m_incrementer.reverseIncrementer());

    // Launcher Controls:
    // operPad.a().whileTrue(m_launcher.runLauncher());
    // operPad.x().whileTrue(m_launcher.reverseLauncher());

    // Mast Controls:
    // operPad.leftStick().whileTrue(m_mast.mastUpDown(operPad.getLeftX()));
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
