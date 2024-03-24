package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  public enum Task {
    INTAKING("Intaking", 0.0),
    LAUNCHMAN("Launch Manual", 1.0),
    LAUNCHAUTO("Launch Auto", 0.0),
    PUTAMP("Note->Amp", 0.0),
    PUTRAP("Note->Trap", 0.0),
    CLEARJAM("Clear", 0.0),
    IDLE("Idle", 0.0),
    CLIMBING("Climbing", 1.0),
    LOWERING("Lowering", -1.0);

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
  TalonFXConfiguration configFX;

  // used when launcher is in auto mode
  private double launcherSpeedAuto;

  // curent launcher task
  private Task currentTask;

  public Climber() {

    // Startup in Idle
    currentTask = Task.IDLE;

    // Construct Motors
    climber = new TalonFX(50);

    // configure motor(s)
    climber.setInverted(true);

    // Define what signals we need from the Talon(s)
    // BaseStatusSignal.setUpdateFrequencyForAll(
    // 10, climber.getPosition(), climber.getVelocity(), climber.getMotorVoltage());

    // // Don't send data over the canbus anything except defined above.
    // climber.optimizeBusUtilization();

    // TODO: Enable Voltage Compensation for Climber

    // Configrue and share between Motors
    // configFX = new TalonFXConfiguration();

    // configFX.Voltage.PeakForwardVoltage = 11;
    // configFX.Voltage.PeakReverseVoltage = -11;

    // configFX.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    // configFX.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    // configFX.CurrentLimits.StatorCurrentLimit = 40.00;
    // configFX.CurrentLimits.StatorCurrentLimitEnable = true;

    // // Apply Motor Configs
    // StatusCode status = StatusCode.StatusCodeNotInitialized;

    // for (int i = 0; i < 5; ++i) {
    // status = climber.getConfigurator().apply(configFX);
    // if (status.isOK()) break;
    // }
    // if (!status.isOK()) {
    // System.err.println("Could not apply configs, to Climber error code: " +
    // status.toString());
    // }

    // TODO: Impliment Automatic Speed Control
    launcherSpeedAuto = kClimberSpeed;

    System.out.println("*********** Climber Constructed!!");
  }

  // Sets the speed of the lead motor open loop
  public void setClimberSpeedOL(double speed) {
    if (currentTask == Task.CLIMBING && RobotContainer.m_climbLimitSensors.get())
    climber.set(0);
    else {
      climber.set(-speed);
    }
  }

  // Sets the speed of the lead motor to 0
  public void stop() {
    currentTask = Task.IDLE;
    climber.set(0);
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

  // runs when no commands are active
  public Command defaultCommand() {
    return this.run(
        () -> {
          setClimberSpeedOL(currentTask.getSpeed());
          // setLauncherSpeedCL(currentTask.getRPM());
        });
  }

  // Runs with Periodic Thread
  @Override
  public void periodic() {

    //If we hit the limit go back to IDLE
    if (currentTask == Task.CLIMBING && RobotContainer.m_climbLimitSensors.get()){
      currentTask = Task.IDLE;
      }

    // Update Current Task
    setClimberSpeedOL(currentTask.getSpeed());

  

    

    // Log Status
    Logger.recordOutput("Climber/Output", climber.get());
    Logger.recordOutput("Climber/Current_Tsk", currentTask.getTaskName());
  }

  // END OF Lancher Class
}
