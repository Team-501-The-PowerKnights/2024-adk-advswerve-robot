package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.kLauncherLeft;
import static frc.robot.Constants.LauncherConstants.kLauncherRight;
import static frc.robot.Constants.LauncherConstants.kLauncherSpeed;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {

  public enum Task {
    INTAKING("Intaking", 0.0, 0.0),
    LOADINC("Load INC", 0.0, 0.0),
    LAUNCHSUB("Launch Subwoofer", 0.41, 3000.00),
    LAUNCHKEY("Launch Protected", 0.67, 4000),
    LAUNCHPASS("Launch Pass", 1.00, 5000),
    LAUNCHAUTO("Launch Auto", 0.0, 0.0),
    PUTAMP("Note->Amp", 0.0, 0.0),
    LAUNCHTRAP("Launch Tap", 0.3, 1000),
    CLEARJAM("Clear", 0.25, 1000.0),
    IDLE("Idle", 0.0, 0.0),
    LAUCNHNOTE1("Launch Note 1", .67, 55.0),
    LAUCNHNOTE2("Launch Note 2", .55, 45.0);

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
  private StatusSignal<Double> launcherVelocityLeft;
  private StatusSignal<Double> launcherVelocityRight;

  // used when launcher is in auto mode
  private double launcherSpeedAuto;

  // curent launcher task
  private static Task currentTask;

  // Timer for Luancher Spin Time
  private static Timer timer;

  private static Launcher myInstance;

  public Launcher() {

    // Startup in Idle
    currentTask = Task.IDLE;
    timer = new Timer();

    // Construct Motors
    launcherLeft = new TalonFX(kLauncherLeft);
    launcherRight = new TalonFX(kLauncherRight);

    // RPM Data
    launcherVelocityLeft = launcherLeft.getVelocity();
    launcherVelocityRight = launcherRight.getVelocity();

    // configure motors
    // One must be inverted
    // launcherLeft.setInverted(false);
    // launcherRight.setInverted(true);

    // Define what signals we need from the Talon(s)
    BaseStatusSignal.setUpdateFrequencyForAll(
        10, launcherLeft.getPosition(), launcherLeft.getVelocity(), launcherLeft.getMotorVoltage());
    BaseStatusSignal.setUpdateFrequencyForAll(
        10,
        launcherRight.getPosition(),
        launcherRight.getVelocity(),
        launcherRight.getMotorVoltage());

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
      if (statusL.isOK() && statusR.isOK()) break;
    }
    if (!statusL.isOK()) {
      System.out.println(
          "Could not apply configs, to Left Launcher error code: " + statusL.toString());
    }
    if (!statusR.isOK()) {
      System.out.println(
          "Could not apply configs, to Right Launcher error code: " + statusR.toString());
    }

    // TODO: Impliment Automatic Speed Control
    launcherSpeedAuto = kLauncherSpeed;

    Launcher.myInstance = this;

    System.out.println("Launcher Constructed!!");
  }

  public static Launcher getInstance() {
    return Launcher.myInstance;
  }

  /// TODO: Impliment PID Launcher Velocity
  public void setLauncherSpeedCL(double rpm) {
    // launcherLeft.set(-speed);
    // launcherRight.set(-speed);
  }

  // Sets the speed of the lead motor open loop
  public void setLauncherSpeedOL(double speed) {
    launcherLeft.set(speed);
    launcherRight.set(speed);
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

  // Use this command will command the Launcher to Do Something and goes to idle
  // when button
  // released
  public Command setTask(Task task) {
    return this.runOnce(
        () -> {
          currentTask = task; // let subsystem know current task
        });
  }

  // Until we can get closed loop running just make sure motors are spinning
  // "fast" before
  // incrimenting into launcher
  public boolean atSpeed() {
    // Real Robot
    if ((launcherVelocityLeft.getValue() * 60) > 1500 && RobotBase.isSimulation()) {
      return true;
    } else {
      // for Simulation
      if (RobotBase.isReal() && timer.get() > 1.0) {
        return true;
      } else {
        return false;
      }
    }
  }

  @Override
  public void periodic() {

    setLauncherSpeedOL(currentTask.getSpeed());

    if (currentTask == Task.IDLE) {
      timer.stop();
      timer.reset();
    } else {
      if (timer.get() == 0.0) {
        timer.start();
      }

      if (timer.get() > 3) {
        currentTask = Task.IDLE;
      }
    }

    Logger.recordOutput("Launch/Left_Enc", launcherVelocityLeft.getValue() * 60);
    Logger.recordOutput("Launch/Right_Enc", launcherVelocityRight.getValue() * 60);
    Logger.recordOutput("Launch/LeftMotorOutput", launcherLeft.get());
    Logger.recordOutput("Launch/RightMotorOutput", launcherRight.get());
    Logger.recordOutput("Launch/Current_Tsk", currentTask.getTaskName());
    Logger.recordOutput("Launch/Timer", timer.get());
    Logger.recordOutput("Launch/AtSpeed", atSpeed());
  }

  // END OF Lancher Class
}
