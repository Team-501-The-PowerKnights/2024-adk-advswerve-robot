package frc.robot.subsystems;

import static frc.robot.Constants.MastConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.math.BigDecimal;
import java.math.RoundingMode;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

// @AutoLog
public class Mast extends SubsystemBase {

  /// TODO: Sync with ABS Encoder
  // Mast at StartPos Abs=57.5, Rel= 108.0
  // Mast at 0 Deg AbS=108, Rel= -54.0 = 5deg (cancels out
  // Mast at 90 Deg Abs=16.0, Rel= 43.00

  public enum Task {
    INTAKING("Intaking", 32.0),
    LAUNCHSUB("Launch Subwoofer", 40.0),
    LAUCNHKEY("Launch Key", 64.5),
    LAUCNHPASS("Launch Pass", 61.0),
    LAUNCHAUTO("Launch Auto", 6.0),
    PUTAMP("PutAmp", -26.0), // tested
    PUTTRAP("PutTrap", 37.5), // was 40
    CLEARJAM("Clear", 40.0),
    CLIMBING("Climbing", -7.0),
    TESTING("Testing", 0.0),
    IDLE("Idle", 0.0),
    OFFKICKSTAND("Off Kickstand", 50.00),
    BUMPUP("Bump Up", 1.0),
    BUMPDOWN("Bump Down", -1.0),
    LAUCNHNOTEPRE("Launch Preload", 44.0),
    LAUCNHNOTE1("Launch Note 1", 60.0),
    LAUCNHNOTE2("Launch Note 2", 55.0);

    private final String taskName;
    private final double angle;

    Task(String taskName, double angle) {
      this.taskName = taskName;
      this.angle = angle;
    }

    public String getTaskName() {
      return taskName;
    }

    public double getAngle() {
      return this.angle;
    }
  }

  CANSparkMax mastLeft;

  double mastSpeed;
  RelativeEncoder relMastLeftEncoder;
  AbsoluteEncoder absMastLeftEncoder;

  private static double relEncoderDeg;
  private static double absEncOffset;
  private static double testingAngle;

  public static double mastKp;
  public static double mastKi;
  public static double mastKd;
  public static double mastFF;
  public static double mastMaxPosOut;
  public static double mastMaxNegOut;
  public static double mastSetpoint;

  SparkPIDController mastLeftPIDController;

  CommandXboxController operPad;

  LoggedDashboardNumber testingAngleNumber;
  private int counts = 0; // used for encoder sync

  private static Task currentTask;
  private static final double gearRatio = (48.0 / 32.0) * 125.0;

  // TODO: Populate values with encoder values

  public Mast() {

    currentTask = Task.OFFKICKSTAND;

    relEncoderDeg = 0.0;
    testingAngle = 0.0;

    mastKp = 0.88;
    mastKi = 0.00;
    mastKd = 1.0;
    double mastIWind = 150;
    mastFF = 0.0;
    mastMaxPosOut = 0.85;
    mastMaxNegOut = -0.85;
    mastSetpoint = 0.0;

    mastLeft = new CANSparkMax(kMastLeft, MotorType.kBrushless);
    mastLeft.restoreFactoryDefaults();
    mastLeft.setSmartCurrentLimit(10);
    mastLeft.enableVoltageCompensation(10.0);
    mastLeft.setIdleMode(IdleMode.kBrake); // Turn on the brake for PID
    mastLeft.setInverted(false);

    absMastLeftEncoder = mastLeft.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    absEncOffset = 203.0; // Offset in Degrees bsMastLeftEncoder.getPosition();
    absMastLeftEncoder.setInverted(true);

    relMastLeftEncoder = mastLeft.getEncoder();

    mastLeftPIDController = mastLeft.getPIDController();
    // set PID coefficients
    mastLeftPIDController.setP(mastKp);
    mastLeftPIDController.setI(mastKi);
    mastLeftPIDController.setD(mastKd);
    mastLeftPIDController.setIZone(mastIWind); // windup I
    mastLeftPIDController.setFF(mastFF);
    mastLeftPIDController.setOutputRange(mastMaxNegOut, mastMaxPosOut);

    testingAngleNumber = new LoggedDashboardNumber("Mast/Test_Angle", testingAngle);
    double ABStoRel = getAbsoluteEncoderDegrees() * gearRatio / 360;
    relMastLeftEncoder.setPosition(-ABStoRel);

    System.out.println("Mast Constructed!!");
  }

  private void setMastPID(double setPoint) {
    mastSetpoint = (setPoint * gearRatio / 360);
    mastLeftPIDController.setReference(mastSetpoint, ControlType.kPosition);
    // System.out.println(mastSetpoint);
  }

  private double getAbsoluteEncoderDegrees() {
    // Read ABS Encoder
    return (absMastLeftEncoder.getPosition() * 360) - absEncOffset;
  }

  public Command setTask(Task task) {

    return this.runOnce(
        () -> {
          currentTask = task;
        });
  }

  // Hopefully this runs once constructed every 20ms...
  @Override
  public void periodic() {
    /// TODO: Offer Bump Command
    /*
     * if(currentTask == Task.BUMPUP){
     *
     * }
     *
     * if(currentTask == Task.BUMPDOWN){
     *
     * }
     */

    // Update Pid Angle
    if (currentTask == Task.TESTING) {
      setMastPID(testingAngle);
    } else {
      setMastPID(currentTask.getAngle());
    }

    // Update Dashboard with Mast Task
    // SmartDashboard.putString("Mast/Task", currentTask.taskName);

    // Get Datafrom Sensors
    relEncoderDeg =
        (relMastLeftEncoder.getPosition() * 360 / gearRatio); // + mastStartingAngleOffset;

    // find ABS vs Relative Error
    double absEncoderDeg = getAbsoluteEncoderDegrees();
    double error = relEncoderDeg - absEncoderDeg;

    if (++counts >= 30) {
      if (Math.abs(error) > .4) {
        // System.out.println("*** fixing at " + counts);
        double absToRel =
            new BigDecimal(absEncoderDeg * gearRatio / 360)
                .setScale(3, RoundingMode.DOWN)
                .doubleValue();
        relMastLeftEncoder.setPosition(absToRel);
      } else {
        // System.out.println("*** not fixing at " + counts);
      }
      counts = 0;
    } else {
      // System.out.println("skipping at " + counts);
    }

    // SmartDashboard.putNumber("Mast/Left_Enc", leftEncAngle);
    // SmartDashboard.putNumber("Mast/Right_Enc", rightEncAngle);
    // SmartDashboard.putNumber("Mast/Abs_Enc", absEncAngle);

    // Uses Logger to log dashboard value and create dashboard field
    testingAngle = testingAngleNumber.get();

    Logger.recordOutput("Mast/Rel_Enc", relEncoderDeg);
    Logger.recordOutput("Mast/Abs_Enc", absEncoderDeg);
    Logger.recordOutput("Mast/ErrorABS", error);
    Logger.recordOutput("Mast/LeftMotorOutput", mastLeft.get());
    Logger.recordOutput("Mast/Target", currentTask.getAngle());
    Logger.recordOutput("Mast/Current_Tsk", currentTask.getTaskName());
  }

  // END OF Mast Class
}
