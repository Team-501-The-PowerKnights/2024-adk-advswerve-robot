package frc.robot.subsystems;

import static frc.robot.Constants.FeederConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

  // CANSparkMax feederLeft;

  public enum Task {
    INTAKING("Intaking", 1.0),
    LAUNCHMAN("Launching", 0.00),
    LAUNCHAUTO("Launching", 0.00),
    PUTAMP("PutAmp", 0.00),
    PUTRAP("PutTrap", 0.00),
    CLEARJAM("Clear", -1.00),
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

  CANSparkMax feederRight;
  double feederSpeed;
  DigitalInput feederSensor;
  DigitalGlitchFilter feederSensorFilter;
  Task currentTask;

  public Feeder() {
    feederRight = new CANSparkMax(kFeederRight, MotorType.kBrushless);
    feederRight.restoreFactoryDefaults();

    feederRight.setSmartCurrentLimit(kFeederCurrentLimit);
    feederRight.setInverted(true);
    feederSpeed = kFeederSpeed;

    // Note Detection Sensor
    feederSensor = new DigitalInput(0);
    feederSensorFilter = new DigitalGlitchFilter();
    feederSensorFilter.add(feederSensor);
    feederSensorFilter.setPeriodNanoSeconds(50000000); // 50ms constant to filter glitch
    System.out.println("Feeder Constructed!!");
  }
  // Sets the speed of the lead motor
  public void setFeederSpeed(double speed) {
    feederRight.set(speed);
  }
  // Sets the speed of the lead motor to 0
  public void stop() {
    feederRight.set(0);
  }

  // Use this command to run a common subsystem task
  public Command setTask(Task task) {
    return this.startEnd(
        () -> {
          setFeederSpeed(task.getSpeed());
        },
        () -> {
          stop();
        });
  }

  // Use this command to run a common subsystem task
  public Command runFeeder() {
    return this.startEnd(
        () -> {
          setFeederSpeed(kFeederSpeed);
        },
        () -> {
          stop();
        });
  }
  // Use this command to "eject" a note back onto the floor
  public Command reverseFeeder() {
    return this.startEnd(
        () -> {
          setFeederSpeed(kFeederSpeed * -0.5);
        },
        () -> {
          stop();
        });
  }

  // END OF Feeder Class
}
