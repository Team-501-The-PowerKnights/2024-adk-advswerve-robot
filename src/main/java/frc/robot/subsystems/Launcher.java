package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.kLauncherLeft;
import static frc.robot.Constants.LauncherConstants.kLauncherRight;
import static frc.robot.Constants.LauncherConstants.kLauncherSpeed;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

  public enum Task {
    INTAKING("Intaking", 0.0, 0.0),
    LAUNCHMAN("Launch Manual", 1.0, 4000.00),
    LAUNCHAUTO("Launch Auto", 0.0, 0.0),
    PUTAMP("Note->Amp", 0.0, 0.0),
    PUTRAP("Note->Trap", 0.0, 0.0),
    CLEARJAM("Clear", 0.25, 1000.0),
    IDLE("Idle", 0.0, 0.0);

    private final String taskName;
    private final double speed;
    private final double targetRPM;

    Task(String taskName, double speed, double targetRPM) {
      this.taskName = taskName;
      this.speed = speed;
      this.targetRPM = targetRPM;
    }

    public String getTaskName() {
      return taskName;
    }

    public double getSpeed() {
      return this.speed;
    }

    public double getRPM() {
      return this.targetRPM;
    }
  }

  // Motors - Speed Controls
  private TalonFX launcherLeft;
  private TalonFX launcherRight;
  TalonFXConfiguration configFX;

  // used when launcher is in auto mode
  private double launcherSpeedAuto;

  // curent launcher task
  private Task currentTask;

  public Launcher() {

    // Startup in Idle
    currentTask = Task.IDLE;

    // Construct Motors
    launcherLeft = new TalonFX(kLauncherLeft);
    launcherRight = new TalonFX(kLauncherRight);

    // configure motors
    // One must be inverted
    launcherLeft.setInverted(true);
    launcherRight.setInverted(false);

      //Define what signals we need from the Talon(s)
    BaseStatusSignal.setUpdateFrequencyForAll(10,
        launcherLeft.getPosition(),
        launcherLeft.getVelocity(),
        launcherLeft.getMotorVoltage());

    // Don't send data over the canbus anything except defined above.
    launcherLeft.optimizeBusUtilization();
    launcherRight.optimizeBusUtilization();

    // TODO: Enable Voltage Compensation for Launcher

    // Configrue and share between Motors
    configFX = new TalonFXConfiguration();

    configFX.Voltage.PeakForwardVoltage = 11;
    configFX.Voltage.PeakReverseVoltage = -11;

    configFX.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configFX.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    // Apply Motor Configs
    StatusCode statusL = StatusCode.StatusCodeNotInitialized;
    StatusCode statusR = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusL = launcherLeft.getConfigurator().apply(configFX);
      statusR = launcherRight.getConfigurator().apply(configFX);
      if (statusL.isOK() && statusR.isOK())
        break;
    }
    if (!statusL.isOK()) {
      System.out.println("Could not apply configs, to Left Launcher error code: " + statusL.toString());
    }
    if (!statusR.isOK()) {
      System.out.println("Could not apply configs, to Right Launcher error code: " + statusR.toString());
    }

    // TODO: Impliment Automatic Speed Control
    launcherSpeedAuto = kLauncherSpeed;

    System.out.println("Launcher Constructed!!");
  }

  /// TODO: Impliment PID Launcher Velocity
  public void setLauncherSpeedCL(double rpm) {
    // launcherLeft.set(-speed);
    // launcherRight.set(-speed);
  }

  // Sets the speed of the lead motor open loop
  public void setLauncherSpeedOL(double speed) {
    launcherLeft.set(-speed);
    launcherRight.set(-speed);

  }

  // Sets the speed of the lead motor to 0
  public void stop() {
    currentTask = Task.IDLE;
    launcherLeft.set(0);
    launcherRight.set(0);
  }

  // Use this command will command the Launcher to Do Something and goes to idle
  // when button
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

  // runs when no commands are active
  public Command defaultCommand() {
    return this.run(
        () -> {
          setLauncherSpeedOL(currentTask.getSpeed());
          // setLauncherSpeedCL(currentTask.getRPM());
        });
  }

  // END OF Lancher Class
}
