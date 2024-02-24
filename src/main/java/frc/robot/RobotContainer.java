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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.flywheels.FlywheelsIOSparkMax;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersSensorsIO;
import frc.robot.subsystems.rollers.RollersSensorsIOReal;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.rollers.feeder.FeederIOSim;
import frc.robot.subsystems.rollers.feeder.FeederIOSparkMax;
import frc.robot.subsystems.rollers.indexer.Indexer;
import frc.robot.subsystems.rollers.indexer.IndexerIOSim;
import frc.robot.subsystems.rollers.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.rollers.intakeFront.IntakeFront;
import frc.robot.subsystems.rollers.intakeFront.IntakeFrontIOSim;
import frc.robot.subsystems.rollers.intakeFront.IntakeFrontIOSparkMax;
import frc.robot.subsystems.rollers.intakeRear.IntakeRear;
import frc.robot.subsystems.rollers.intakeRear.IntakeRearIOSim;
import frc.robot.subsystems.rollers.intakeRear.IntakeRearIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private Rollers rollers;
  private Feeder feeder;
  private Indexer indexer;
  private IntakeFront intakeFront;
  private IntakeRear intakeRear;
  private Flywheels flywheels;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Declare component subsystems (not visible outside constructor)

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(0), // FL
                new ModuleIOSparkMax(1), // FR
                new ModuleIOSparkMax(2), // BL
                new ModuleIOSparkMax(3)); // BR

        feeder = new Feeder(new FeederIOSparkMax());
        indexer = new Indexer(new IndexerIOSparkMax());
        intakeFront = new IntakeFront(new IntakeFrontIOSparkMax());
        intakeRear = new IntakeRear(new IntakeRearIOSparkMax());
        rollers = new Rollers(feeder, indexer, intakeFront, intakeRear, new RollersSensorsIOReal());

        flywheels = new Flywheels(new FlywheelsIOSparkMax());
        // drive = new Drive(
        // new GyroIOPigeon2(true),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        flywheels = new Flywheels(new FlywheelsIOSparkMax());
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

        flywheels = new Flywheels(new FlywheelsIOSim());

        feeder = new Feeder(new FeederIOSim());
        indexer = new Indexer(new IndexerIOSim());
        intakeFront = new IntakeFront(new IntakeFrontIOSim());
        intakeRear = new IntakeRear(new IntakeRearIOSim());
        rollers = new Rollers(feeder, indexer, intakeFront, intakeRear, new RollersSensorsIO() {});
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

    // Set up auto routines
    // NamedCommands.registerCommand(
    // "Run Flywheel",
    // Commands.startEnd(
    // () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
    // flywheel)
    // .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Flywheel SysId (Quasistatic Forward)",
    // flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Flywheel SysId (Quasistatic Reverse)",
    // flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Flywheel SysId (Dynamic Forward)",
    // flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Flywheel SysId (Dynamic Reverse)",
    // flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  // STU
  /*
   * private static class ZeroSupplier {
   * public static double zero() {
   * return 0.0;
   * }
   * }
   */
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
            () -> (-driver.getLeftY() * 0.50),
            () -> (-driver.getLeftX() * 0.50),
            () -> (driver.getRightX() * 0.50)));
    /*
        driver
            .button(1)
            .whileTrue(
                Commands.startEnd(
                    () -> flywheels.runVelocity(flywheelSpeedInput.get()), flywheels::IDLE, flywheels));
    */
    // driver.button(2).whileTrue( () - > ).runEnd(flywheel.runVolts(.5), flywheel.runVolts(0) ));

    // drive.setDefaultCommand(
    // DriveCommands.joystickDrive(
    // drive,
    // () -> -controller.getLeftY(),
    // () -> ZeroSupplier.zero(),
    // () -> ZeroSupplier.zero()));
    // DriveCommands.joystickDrive(
    // drive,
    // () -> ZeroSupplier.zero(),
    // () -> ZeroSupplier.zero(),
    // () -> ZeroSupplier.zero()));
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller
    // .b()
    // .onTrue(
    // Commands.runOnce(
    // () ->
    // drive.setPose(
    // new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    // drive)
    // .ignoringDisable(true));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
