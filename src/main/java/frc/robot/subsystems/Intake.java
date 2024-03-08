package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
 
  public enum Task {
    INTAKING("Intaking", 1.0),
    LAUNCHMAN("LaunchMan-Idle", 0.00),
    LAUNCHAUTO("LaunchAuto-Idle", 0.0),
    PUTAMP("PutAmp-Idle", 0.0),
    PUTTRAP("PutTrap-Idle", 0.0),
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

  //current intake task
  private Task currentTask;

  public Intake() {

 // Startup in Idle
 currentTask = Task.IDLE;


    intakeFront = new CANSparkMax(kIntakeFront, MotorType.kBrushless);
    intakeFront.restoreFactoryDefaults();
    intakeRear = new CANSparkMax(kIntakeRear, MotorType.kBrushless);
    intakeRear.restoreFactoryDefaults();

    intakeFront.setSmartCurrentLimit(kIntakeCurrentLimit);
    intakeRear.setSmartCurrentLimit(kIntakeCurrentLimit);

// curent launcher task


    System.out.println("Intake Constructed!!");
  }

  // Sets the speed of the lead motor
  public void setIntakeSpeed(double speed) {
    intakeFront.set(speed);
    intakeRear.set(-speed);
  }

  // Sets the speed of the lead motor to 0
  public void stop() {
    intakeFront.set(0);
    intakeRear.set(0);
  }

  // Use this command to pull a note off the floor
  public Command runIntake() {
    return this.startEnd(
        () -> {
          setIntakeSpeed(kIntakeSpeed);
        },
        () -> {
          stop();
        });
  }

  // Use this command to "eject" a note back onto the floor
  public Command reverseIntake() {
    return this.startEnd(
        () -> {
          setIntakeSpeed(kIntakeSpeed * -0.5);
        },
        () -> {
          stop();
        });
  }

  // Use this command to pull a note off the floor
  public Command setTask(Task task) {

    return this.startEnd(
      () -> {
        currentTask = task; // let subsystem know current task
      },
      () -> {
        currentTask = Task.IDLE;
      });
  }

   //runs when no commands are active
   public Command defaultCommand() {
    return this.run(
        () -> {
          setIntakeSpeed(currentTask.getSpeed());
          // setLauncherSpeedCL(currentTask.getRPM());
        });
  }
  // END OF INTAKE CLASS
}
