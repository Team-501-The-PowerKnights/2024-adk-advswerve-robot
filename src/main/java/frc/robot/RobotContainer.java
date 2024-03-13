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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.Task;
import frc.robot.subsystems.Incrementer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Mast;
import frc.robot.subsystems.TopFeederSensor;
import frc.robot.subsystems.TopIncrementerSensor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
import frc.robot.subsystems.drive.ModuleIOSparkMax;

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
  private final Climber m_climber;
  public static final TopFeederSensor m_topFeederSensor = new TopFeederSensor();
  public static final TopIncrementerSensor m_topIncrementerSensor = new TopIncrementerSensor();
  ;

  // Controller
  private final CommandXboxController driverPad = new CommandXboxController(0);
  private final CommandXboxController operPad = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        switch (Constants.currentRobot) {
          case PROTO:
            drive =
                new Drive(
                    new GyroIOPigeon2(false),
                    new ModuleIOSparkMax(0), // FL
                    new ModuleIOSparkMax(1), // FR
                    new ModuleIOSparkMax(2), // BL
                    new ModuleIOSparkMax(3)); // BR

            m_intake = null;
            m_feeder = null;
            m_incrementer = null;
            m_launcher = null;
            m_climber = null;
            m_mast = null;
            break;

          case REAL:
            drive =
                new Drive(
                    new GyroIOPigeon2(false),
                    new ModuleIOSparkFlex(0), // FL
                    new ModuleIOSparkFlex(1), // FR
                    new ModuleIOSparkFlex(2), // BL
                    new ModuleIOSparkFlex(3)); // BR

            m_intake = new Intake();
            m_feeder = new Feeder();
            m_incrementer = new Incrementer();
            m_launcher = new Launcher();
            m_mast = new Mast();
            m_climber = new Climber();
            break;

          case SUITCASE:
          default:
            drive =
                new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});

            m_intake = null;
            m_feeder = null;
            m_incrementer = null;
            m_launcher = null;
            m_mast = null;
            m_climber = null;
            break;
        }
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
        m_intake = new Intake();
        m_feeder = new Feeder();
        m_incrementer = new Incrementer();
        m_launcher = new Launcher();
        m_mast = new Mast();
        m_climber = new Climber();
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
        m_intake = null;
        m_feeder = null;
        m_incrementer = null;
        m_launcher = null;
        m_mast = null;
        m_climber = null;
        break;
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    switch (Constants.currentRobot) {
      case PROTO:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> (MathUtil.applyDeadband(driverPad.getLeftY() * .5, .07)),
                () -> (MathUtil.applyDeadband(driverPad.getLeftX() * .5, .07)),
                () -> (MathUtil.applyDeadband(-driverPad.getRightX() * 0.5, .07))));

        break;

      case REAL:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> (MathUtil.applyDeadband(driverPad.getLeftY() * .85, .07)),
                () -> (MathUtil.applyDeadband(driverPad.getLeftX() * .85, .07)),
                () -> (MathUtil.applyDeadband(-driverPad.getRightX() * 0.85, .07))));

        // Intake Note and Load into Launcher
        driverPad
            .leftTrigger()
            .whileTrue(
                Commands.race(
                    m_intake.setTask(Intake.Task.IDLE),
                    m_feeder.setTask(Feeder.Task.IDLE),
                    m_mast.setTask(Mast.Task.PUTTRAP),
                    m_incrementer.setTask(Incrementer.Task.IDLE),
                    m_launcher.setTask(Launcher.Task.IDLE),
                    m_climber.setTask(Climber.Task.CLIMBING)));

        // Intake Note and Load into Launcher
        driverPad
            .rightTrigger()
            .whileTrue(
                Commands.sequence(
                    m_intake.setTask(Intake.Task.INTAKING),
                    m_feeder.setTask(Feeder.Task.INTAKING),
                    m_mast.setTask(Mast.Task.INTAKING),
                    new WaitUntilCommand(m_topFeederSensor::get),
                    m_intake.setTask(Intake.Task.IDLE),
                    m_feeder.setTask(Feeder.Task.IDLE),
                    m_incrementer.setTask(Incrementer.Task.INTAKING),
                    new WaitUntilCommand(m_topIncrementerSensor::get),
                    m_launcher.setTask(Launcher.Task.IDLE)));

        driverPad
            .a()
            .whileTrue(
                Commands.sequence(
                    m_mast.setTask(Mast.Task.INTAKING),
                    m_intake.setTask(Intake.Task.INTAKING),
                    m_feeder.setTask(Feeder.Task.INTAKING),
                    new WaitUntilCommand(m_topFeederSensor::get),
                    m_feeder.setTask(Task.IDLE),
                    m_feeder.setTask(Feeder.Task.IDLE)));

        driverPad
            .b()
            .whileTrue(
                Commands.sequence(
                    m_mast.setTask(Mast.Task.INTAKING),
                    new WaitUntilCommand(m_topFeederSensor::get),
                    m_feeder.setTask(Feeder.Task.TRANSFER),
                    m_incrementer.setTask(Incrementer.Task.TRANSFER),
                    new WaitUntilCommand(m_topIncrementerSensor::get),
                    m_feeder.setTask(Feeder.Task.IDLE),
                    m_incrementer.setTask(Incrementer.Task.IDLE)));

        // Intake Note and Load into Launcher
        operPad
            .leftTrigger()
            .whileTrue(
                Commands.race(
                    m_intake.setTask(Intake.Task.INTAKING),
                    m_feeder.setTask(Feeder.Task.INTAKING),
                    m_mast.setTask(Mast.Task.INTAKING),
                    m_incrementer.setTask(Incrementer.Task.INTAKING),
                    m_launcher.setTask(Launcher.Task.INTAKING)));

        // Launch Note at Speaker Manually from Stage Key
        operPad
            .a()
            .whileTrue(
                Commands.race(
                    m_intake.setTask(Intake.Task.LAUNCHMAN),
                    m_feeder.setTask(Feeder.Task.LAUNCHMAN),
                    m_mast.setTask(Mast.Task.LAUNCHMAN),
                    m_incrementer.setTask(Incrementer.Task.LAUNCHMAN),
                    m_launcher.setTask(Launcher.Task.LAUNCHMAN)));

        // Eject Note at AMP
        operPad
            .b()
            .whileTrue(
                Commands.race(
                    m_intake.setTask(Intake.Task.PUTAMP),
                    m_feeder.setTask(Feeder.Task.PUTAMP),
                    m_mast.setTask(Mast.Task.PUTAMP),
                    m_incrementer.setTask(Incrementer.Task.PUTAMP),
                    m_launcher.setTask(Launcher.Task.PUTAMP)));

        // Clear Jammed System
        operPad
            .leftBumper()
            .and(operPad.rightBumper())
            .whileTrue(
                Commands.race(
                    m_intake.setTask(Intake.Task.CLEARJAM),
                    m_feeder.setTask(Feeder.Task.CLEARJAM),
                    m_mast.setTask(Mast.Task.CLEARJAM),
                    m_incrementer.setTask(Incrementer.Task.CLEARJAM),
                    m_launcher.setTask(Launcher.Task.CLEARJAM)));

        // Mast Controls:
        // operPad.leftStick().whileTrue(m_mast.mastUpDown(operPad.getLeftY()));
        // Do we need to define defaults if we have @ periodic??
        // m_mast.setDefaultCommand(m_mast.mastUpDown(-operPad.getLeftY(), operPad));
        // m_launcher.setDefaultCommand(m_launcher.defaultCommand());

        break;

      case SUITCASE:
      default:
        break;
    }
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
