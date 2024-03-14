package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public enum Task {
    INTAKING("Intaking", 1.0),
    LAUNCHMAN("LaunchMan-Idle", 0.00),
    LAUNCHAUTO("LaunchAuto-Idle", 0.0),
    PUTAMP("PutAmp-Idle", 0.0),
    PUTTRAP("PutTrap-Idle", 0.0),
    TRANSFER("Transfer", 0.0),
    CLEARJAM("Clearing", -1.00),
    IDLE("Idle", 0.0);

    private final String taskName;
    private final double speed;

    Task(String taskName, double speed) {
      this.taskName = taskName;
      this.speed = speed;
    }

    public String getTaskName() {
      return taskName;
    }

    public double getSpeed() {
      return this.speed;
    }
  }

  // Motors - Speed Controls
  CANSparkMax intakeFront;
  CANSparkMax intakeRear;
  double intakeSpeed;

  // current intake task
  private static Task currentTask;

  public Intake() {

    // Startup in Idle
    currentTask = Task.IDLE;

    intakeFront = new CANSparkMax(kIntakeFront, MotorType.kBrushless);
    intakeFront.restoreFactoryDefaults();
    intakeRear = new CANSparkMax(kIntakeRear, MotorType.kBrushless);
    intakeRear.restoreFactoryDefaults();

    intakeFront.setSmartCurrentLimit(kIntakeCurrentLimit);
    intakeRear.setSmartCurrentLimit(kIntakeCurrentLimit);

    // Reduce canbus chatter
    intakeFront.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    intakeFront.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000);
    intakeFront.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000);
    intakeFront.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);
    intakeFront.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);
    intakeFront.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10000);
    intakeFront.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);

    intakeRear.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    intakeRear.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000);
    intakeRear.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000);
    intakeRear.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);
    intakeRear.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);
    intakeRear.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10000);
    intakeRear.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);

    System.out.println("Intake Constructed!!");
  }

  // Sets the speed of the lead motor
  public void setIntakeSpeed(double speed) {
    intakeFront.set(speed);
    intakeRear.set(-speed);
  }

  // Use this command to pull a note off the floor manual control

  // Command Idles System when Letting go of button
  public Command setTaskEnd(Task task) {

    return this.startEnd(
        () -> {
          currentTask = task; // let subsystem know current task
        },
        () -> {
          currentTask = Task.IDLE;
        });
  }

  // Fire and Forget Command
  public Command setTask(Task task) {
    return this.runOnce(
        () -> {
          currentTask = task;
        });
  }

  // Runs with Periodic Thread
  @Override
  public void periodic() {
    // Update Current Task
    setIntakeSpeed(currentTask.getSpeed());

    // If the FeederSensor or IncrementerSensor is on got back to idle.
    if (RobotContainer.m_topFeederSensor.get() || RobotContainer.m_topIncrementerSensor.get()) {
      currentTask = Task.IDLE;
    }

    // Log Status
    Logger.recordOutput("Intake/Output_Front", intakeFront.get());
    Logger.recordOutput("Intake/Output_Rear", intakeRear.get());
    Logger.recordOutput("Intake/Current_Tsk", currentTask.getTaskName());
  }
  // END OF INTAKE CLASS
}
