package frc.robot.subsystems;

import static frc.robot.Constants.FeederConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  // CANSparkMax feederLeft;

  public enum Task {
    INTAKING("Intaking", 0.5),
    LAUNCHMAN("Launching", 0.00),
    LAUNCHAUTO("Launching", 0.00),
    PUTAMP("PutAmp", 0.00),
    PUTRAP("PutTrap", 0.00),
    CLEARJAM("Clear", -1.00),
    TRANSFER("Transfer", 0.40),
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

  CANSparkMax feeder;
  double feederSpeed;

  // Keeps the current task
  private static Task currentTask;

  public Feeder() {
    // Construct Motors
    feeder = new CANSparkMax(kFeederRight, MotorType.kBrushless);
    feeder.restoreFactoryDefaults();

    feeder.setSmartCurrentLimit(kFeederCurrentLimit);
    feeder.setInverted(false);

    // Reduce canbus chatter
    feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000);
    feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000);
    feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);
    feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);
    feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10000);
    feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);

    feederSpeed = kFeederSpeed;

    currentTask = Task.IDLE;

    System.out.println("Feeder Constructed!!");
  }
  // Sets the speed of the lead motor
  public void setFeederSpeed(double speed) {
    feeder.set(speed);
  }
  // Sets the speed of the lead motor to 0
  public void stop() {
    feeder.set(0);
  }

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
    setFeederSpeed(currentTask.getSpeed());

    if (currentTask == Task.INTAKING) {
      // If the Feeder Sensor is Intaking and finds the Note go IDLE
      if (RobotContainer.m_topFeederSensor.get()) {
        currentTask = Task.IDLE;
        System.out.println("Note Loaded");
      }
    }

    if (currentTask == Task.TRANSFER) {
      // If the Feeder Sensor is Intaking and finds the Note go IDLE
      if (!RobotContainer.m_topFeederSensor.get() || RobotContainer.m_topIncrementerSensor.get()) {
        currentTask = Task.IDLE;
        System.out.println("Note Transfered");
      }
    }

    // Log Data
    Logger.recordOutput("Feeder/Output", feeder.get());
    Logger.recordOutput("Feeder/Current_Tsk", currentTask.getTaskName());
  }

  // END OF Feeder Class
}
