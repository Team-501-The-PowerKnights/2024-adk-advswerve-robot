package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  public enum Task {
    INTAKING("Intaking", 0.0),
    LAUNCHMAN("Launch Manual", 1.0),
    LAUNCHAUTO("Launch Auto", 0.0),
    PUTAMP("Note->Amp", 0.0),
    PUTRAP("Note->Trap", 0.0),
    CLEARJAM("Clear", 0.0 ),
    IDLE("Idle", 0.0),
    CLIMBING("Climbing", 0.0),
    LOWERING("Lowering", 0.0);

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
  private TalonFX climber;

  // used when launcher is in auto mode
  private double launcherSpeedAuto;

  // curent launcher task
  private Task currentTask;

  public Climber() {

    // Startup in Idle
    currentTask = Task.IDLE;

    // Construct Motors
    climber = new TalonFX(kClimber);
    // One must be inverted
    climber.setInverted(true);
   // climber.StatorCurrentLimitConfiguration(20);

    // TODO: Impliment Automatic Speed Control
    launcherSpeedAuto = kClimberSpeed;

    System.out.println("Launcher Constructed!!");
  }


  // Sets the speed of the lead motor open loop
  public void setLauncherSpeedOL(double speed) {
    climber.set(-speed);
  }

  // Sets the speed of the lead motor to 0
  public void stop() {
    currentTask = Task.IDLE;
    climber.set(0);

  }

  // Use this command will command the Launcher to Do Something and goes to idle when button
  // released
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
          setLauncherSpeedOL(currentTask.getSpeed());
          // setLauncherSpeedCL(currentTask.getRPM());
        });
  }

  // END OF Lancher Class
}
