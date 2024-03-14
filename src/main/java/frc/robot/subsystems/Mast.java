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
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

// @AutoLog
public class Mast extends SubsystemBase {

///TODO: Sync with ABS Encoder
//Mast at StartPos Abs=57.5, Rel= 108.0
//Mast at 0 Deg    AbS=108,  Rel= -54.0 = 5deg (cancles out 
//Mast at 90 Deg   Abs=16.0, Rel= 43.00


  public enum Task {
    INTAKING("Intaking", -17.0),
    LAUNCHSUB("Launch Subwoofer", 6.0),
    LAUCNHKEY("Launch Key", 5.0),
    LAUNCHAUTO("Launch Auto", 6.0),
    PUTAMP("PutAmp", -90.0), //tested
    PUTTRAP("PutTrap", 80.0),
    CLEARJAM("Clear", 10.0),
    CLIMBING("Climbing", -50.0),
    TESTING("Testing", 0.0),
    IDLE("Idle", 0.0),
    OFFKICKSTAND("Off Kickstand",10.0),
    BUMPUP("Bump Up",1.0),
    BUMPDOWN("Bump Down",-1.0);

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
  CANSparkMax mastRight;
  DigitalInput mastSensor;
  DigitalGlitchFilter mastSensorFilter;

  double mastSpeed;
  RelativeEncoder relmastLeftEncoder;
  RelativeEncoder relmastRightEncoder;
  CommandXboxController operPad;
  AbsoluteEncoder absMastLeftEncoder;

  private static double leftEncAngle;
  private static double rightEncAngle;
  private static double AbsEncOffset;
  private static double testingAngle;

  public static double mastKp;
  public static double mastKi;
  public static double mastKd;
  public static double mastFF;
  public static double mastMaxPosOut;
  public static double mastMaxNegOut;
  public static double mastSetpoint;

  SparkPIDController mastLeftPIDController;
  SparkPIDController mastRightPIDController;

  LoggedDashboardNumber testingAngleNumber;

  private static Task currentTask;
  private static final double gearRatio = (48.0 / 32.0) * 25.0 * -1;

  // TODO: Populate values with encoder values

  public Mast() {

    currentTask = Task.LAUCNHKEY;

    leftEncAngle = 0.0;
    rightEncAngle = 0.0;
    testingAngle = 0.0;

    mastKp = 1.0;
    mastKi = 0.001;
    mastKd = 0.0;
    mastFF = 0.0;
    mastMaxPosOut = 0.4;
    mastMaxNegOut = -0.4;
    mastSetpoint = 0.0;

    mastLeft = new CANSparkMax(kMastLeft, MotorType.kBrushless);
    mastLeft.restoreFactoryDefaults();

    mastRight = new CANSparkMax(kMastRight, MotorType.kBrushless);
    mastRight.restoreFactoryDefaults();

    mastLeft.setSmartCurrentLimit(kMastCurrentLimit);
    mastRight.setSmartCurrentLimit(kMastCurrentLimit);

    mastLeft.enableVoltageCompensation(12.0);
    mastRight.enableVoltageCompensation(12.0);

    mastLeft.setIdleMode(IdleMode.kBrake); // Turn on the brake for PID
    mastRight.setIdleMode(IdleMode.kBrake); // Turn off the brake other motor
    mastLeft.setInverted(true);
    mastRight.setInverted(false);
    // mastRight.follow(mastLeft);

    absMastLeftEncoder = mastLeft.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    AbsEncOffset = 54.0;  //Offset in Degrees bsMastLeftEncoder.getPosition();
    absMastLeftEncoder.setInverted(true);

    relmastLeftEncoder = mastLeft.getEncoder();
    relmastRightEncoder = mastRight.getEncoder();

    mastLeftPIDController = mastLeft.getPIDController();
    mastRightPIDController = mastRight.getPIDController();
    // set PID coefficients
    mastLeftPIDController.setP(mastKp);
    mastLeftPIDController.setI(mastKi);
    mastLeftPIDController.setD(mastKd);
    // mastPIDController.setIZone(); //windup I
    mastLeftPIDController.setFF(mastFF);
    mastLeftPIDController.setOutputRange(mastMaxNegOut, mastMaxPosOut);
    mastRightPIDController.setP(mastKp);
    mastRightPIDController.setI(mastKi);
    mastRightPIDController.setD(mastKd);
    // mastPIDController.setIZone(); //windup I
    mastRightPIDController.setFF(mastFF);
    mastRightPIDController.setOutputRange(mastMaxNegOut, mastMaxPosOut);

    testingAngleNumber = new LoggedDashboardNumber("Mast/Test_Angle", testingAngle);

    System.out.println("Mast Constructed!!");
  }


  private void setMastPID(double setPoint) {
    mastSetpoint = setPoint * 37.5 / 360;
    mastLeftPIDController.setReference(mastSetpoint, ControlType.kPosition);
    mastRightPIDController.setReference(mastSetpoint, ControlType.kPosition);
    System.out.println(mastSetpoint);
  }

  private double getAbsoluteEncoderDegrees(){
   //Read ABS Encoder
   return (absMastLeftEncoder.getPosition() * 360) - AbsEncOffset;
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
    ///TODO: Offer Bump Command
    /*
    if(currentTask == Task.BUMPUP){
      
    }

    if(currentTask == Task.BUMPDOWN){
      
    }
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
    leftEncAngle =
        (relmastLeftEncoder.getPosition() * 360 / gearRatio); // + mastStartingAngleOffset;
    rightEncAngle =
        (relmastRightEncoder.getPosition() * 360 / gearRatio); // + mastStartingAngleOffset;
      

    // SmartDashboard.putNumber("Mast/Left_Enc", leftEncAngle);
    // SmartDashboard.putNumber("Mast/Right_Enc", rightEncAngle);
    // SmartDashboard.putNumber("Mast/Abs_Enc", absEncAngle);

    // Uses Logger to log dashboard value and create dashboard field
    testingAngle = testingAngleNumber.get();

    Logger.recordOutput("Mast/Left_Enc", leftEncAngle);
    Logger.recordOutput("Mast/Right_Enc", rightEncAngle);
    Logger.recordOutput("Mast/Abs_Enc", getAbsoluteEncoderDegrees());
    Logger.recordOutput("Mast/LeftMotorOutput", mastLeft.get());
    Logger.recordOutput("Mast/RightMotorOutput", mastRight.get());
    Logger.recordOutput("Mast/Current_Tsk", currentTask.getTaskName());
  }

  // END OF Mast Class
}
