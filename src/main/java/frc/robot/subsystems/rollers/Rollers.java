package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.rollers.indexer.Indexer;
import frc.robot.subsystems.rollers.intakeFront.IntakeFront;
import frc.robot.subsystems.rollers.intakeRear.IntakeRear;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  private final Feeder feeder;
  private final Indexer indexer;
  private final IntakeFront intakeFront;
  private final IntakeRear intakeRear;

  private final RollersSensorsIO sensorsIO;
  private final RollersSensorsIOInputsAutoLogged sensorInputs =
      new RollersSensorsIOInputsAutoLogged();

  public enum Goal {
    IDLE,
    FLOOR_INTAKE,
    STATION_INTAKE,
    EJECT_TO_FLOOR,
    FEED_SHOOTER
  }

  public Rollers(
      Feeder feeder,
      Indexer indexer,
      IntakeFront intakeFront,
      IntakeRear intakeRear,
      RollersSensorsIO sensorsIO) {
    this.feeder = feeder;
    this.indexer = indexer;
    this.intakeFront = intakeFront;
    this.intakeRear = intakeRear;
    this.sensorsIO = sensorsIO;

    setDefaultCommand(runOnce(this::goIdle).withName("RollersIdle"));
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorInputs);
    Logger.processInputs("RollersSensors", sensorInputs);

    if (DriverStation.isDisabled()) {
      goIdle();
    }

    feeder.periodic();
    indexer.periodic();
    intakeFront.periodic();
    intakeRear.periodic();
  }

  public void goIdle() {
    feeder.setGoal(Feeder.Goal.IDLE);
    indexer.setGoal(Indexer.Goal.IDLE);
    intakeFront.setGoal(IntakeFront.Goal.IDLE);
    intakeRear.setGoal(IntakeRear.Goal.IDLE);
  }

  public Command floorIntakeCommand() {
    return startEnd(
            () -> {
              feeder.setGoal(Feeder.Goal.FLOOR_INTAKING);
              indexer.setGoal(Indexer.Goal.FLOOR_INTAKING);
              intakeFront.setGoal(IntakeFront.Goal.FLOOR_INTAKING);
              intakeRear.setGoal(IntakeRear.Goal.FLOOR_INTAKING);
              if (sensorInputs.shooterStaged) {
                indexer.setGoal(Indexer.Goal.IDLE);
              }
            },
            this::goIdle)
        .withName("RollersFloorIntake");
  }

  public Command stationIntakeCommand() {
    return startEnd(
            () -> {
              feeder.setGoal(Feeder.Goal.IDLE);
              indexer.setGoal(Indexer.Goal.STATION_INTAKING);
              intakeFront.setGoal(IntakeFront.Goal.IDLE);
              intakeRear.setGoal(IntakeRear.Goal.IDLE);
              if (sensorInputs.shooterStaged) { // TODO: ADD THIS BANNER
                indexer.setGoal(Indexer.Goal.IDLE);
              }
            },
            this::goIdle)
        .withName("RollersStationIntake");
  }

  public Command ejectFloorCommand() {
    return startEnd(
            () -> {
              feeder.setGoal(Feeder.Goal.EJECTING);
              indexer.setGoal(Indexer.Goal.EJECTING);
              intakeFront.setGoal(IntakeFront.Goal.EJECTING);
              intakeRear.setGoal(IntakeRear.Goal.EJECTING);
            },
            this::goIdle)
        .withName("RollersEjectFloor");
  }

  public Command feedShooterCommand() {
    return startEnd(
            () -> {
              feeder.setGoal(Feeder.Goal.SHOOTING);
              indexer.setGoal(Indexer.Goal.SHOOTING);
              intakeFront.setGoal(IntakeFront.Goal.IDLE);
              intakeRear.setGoal(IntakeRear.Goal.IDLE);
            },
            this::goIdle)
        .withName("RollersFeedShooter");
  }

  public Command idleRollersCommand() {
    return startEnd(
            () -> {
              feeder.setGoal(Feeder.Goal.IDLE);
              indexer.setGoal(Indexer.Goal.IDLE);
              intakeFront.setGoal(IntakeFront.Goal.IDLE);
              intakeRear.setGoal(IntakeRear.Goal.IDLE);
            },
            this::goIdle)
        .withName("RollersIdle");
  }
}
