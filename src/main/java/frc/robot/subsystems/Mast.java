package frc.robot.subsystems;

import static frc.robot.Constants.MastConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// @AutoLog
public class Mast extends SubsystemBase {
  CANSparkMax mastLeft;
  CANSparkMax mastRight;

  double mastSpeed;
  RelativeEncoder relmastLeftEncoder;
  RelativeEncoder relmastRightEncoder;
  CommandXboxController operPad;
  DutyCycleEncoder absMastLeftEncoder;

  public static double mastKp;
  public static double mastKi;
  public static double mastKd;
  public static double mastFF;
  public static double mastMaxPosOut;
  public static double mastMaxNegOut;
  public static double mastSetpoint;

  SparkPIDController mastLeftPIDController;
  SparkPIDController mastRightPIDController;

  private static double mastPosAmp;
  private static double mastPosLoad;
  private static double mastPosSubwoofer;
  private static double mastClimber;
  private static double mastPosCamera;
  private static double mastPosTrap;
  private static double mastAbsAngle;

  // TODO: Populate values with encoder values

  public Mast() {
    mastKp = 1.0;
    mastKi = 0.0;
    mastKd = 0.0;
    mastFF = 0.0;
    mastMaxPosOut = 0.1;
    mastMaxNegOut = -0.1;
    mastSetpoint = 0.0;

    mastPosAmp = 90.0;
    mastPosLoad = 0.0;
    mastPosSubwoofer = 10.0;
    mastClimber = 55.0;
    mastPosCamera = 0.0;
    mastPosTrap = -45.0;

    mastLeft = new CANSparkMax(kMastLeft, MotorType.kBrushless);
    mastLeft.restoreFactoryDefaults();

    mastRight = new CANSparkMax(kMastRight, MotorType.kBrushless);
    mastRight.restoreFactoryDefaults();

    mastLeft.setSmartCurrentLimit(kMastCurrentLimit);
    mastRight.setSmartCurrentLimit(kMastCurrentLimit);

    mastLeft.enableVoltageCompensation(12.0);
    mastRight.enableVoltageCompensation(12.0);

    mastLeft.setIdleMode(IdleMode.kBrake); // Turn on the brake for PID
    mastRight.setIdleMode(IdleMode.kCoast); // Turn off the brake other motor
    mastLeft.setInverted(true);
    mastRight.setInverted(false);
    // mastRight.follow(mastLeft);

    absMastLeftEncoder = new DutyCycleEncoder(2);

    relmastLeftEncoder = mastLeft.getEncoder();
    relmastRightEncoder = mastRight.getEncoder();

    mastLeftPIDController = mastLeft.getPIDController();
    mastRightPIDController = mastRight.getPIDController();
    // set PID coefficients
    mastLeftPIDController.setP(mastKp);
    mastLeftPIDController.setI(mastKi);
    mastLeftPIDController.setD(mastKd);
    // mastPIDController.setIZone();  //windup I
    mastLeftPIDController.setFF(mastFF);
    mastLeftPIDController.setOutputRange(mastMaxNegOut, mastMaxPosOut);
    mastRightPIDController.setP(mastKp);
    mastRightPIDController.setI(mastKi);
    mastRightPIDController.setD(mastKd);
    // mastPIDController.setIZone();  //windup I
    mastRightPIDController.setFF(mastFF);
    mastRightPIDController.setOutputRange(mastMaxNegOut, mastMaxPosOut);

    // mastSpeed = kMastSpeed;//Speed will be controlled by axis from remote
    System.out.println("Mast Constructed!!");
  }

  // Sets the speed of the lead motor
  public void setMastSpeed(double speed) {
    System.out.println("running setMastSpeed = " + speed);
    if (Math.abs(speed) > 0.1) {
      mastLeft.set(speed / 5);
      mastRight.set(speed / 5);
    } else {
      mastLeft.set(0);
      mastRight.set(0);
    }
  }

  // Sets the speed of the lead motor to 0
  public void stop() {
    // mastLeft.set(0);
    mastRight.set(0);
  }

  private void setMastPID(double setPoint) {
    mastSetpoint = -setPoint * 37.5 / 360;
    mastLeftPIDController.setReference(mastSetpoint, ControlType.kPosition);
    mastRightPIDController.setReference(mastSetpoint, ControlType.kPosition);
  }

  // Use this command to set PID to Amp Angle
  public Command setAmpCommand() {
    return this.runOnce(
        () -> {
          setMastPID(mastPosAmp);
        });
  }

  // Use this command to set PID to Speaker Subwoofer
  public Command setSubwooferCommand() {
    return this.runOnce(
        () -> {
          setMastPID(mastPosSubwoofer);
        });
  }

  // Use this command to set PID to Load from Intake
  public Command setLoadingCommand() {
    return this.runOnce(
        () -> {
          setMastPID(mastPosLoad);
        });
  }

  // Use this command to set PID to Score in Trap
  public Command setTrapCommand() {
    return this.runOnce(
        () -> {
          setMastPID(mastPosTrap);
        });
  }

  public Command setClimberCommand() {
    return this.runOnce(
        () -> {
          setMastPID(mastClimber);
          System.out.println("RUNNING MAST CLIMBER" + mastClimber);
        });
  }
  // Adding a coment her to commit changes to Git.
  public Command mastUpDown(double controllerSpeed, CommandXboxController operator) {

    double gearRatioLeft = (48.0 / 32.0) * 25.0 * -1;
    double gearRatioRight = (48.0 / 32.0) * 25.0 * -1;
    // double operLeftY = 0.0;
    return this.run(
        () -> {
          SmartDashboard.putBoolean("Mast.moving", true);
          System.out.println("running mastUpDown = " + operator.getLeftY());
          if (operator.getLeftY() > 0.5 || operator.getLeftY() < -0.5) {

            mastLeftPIDController.setReference(operator.getLeftY(), ControlType.kVoltage);
            mastRightPIDController.setReference(operator.getLeftY(), ControlType.kVoltage);
          }
          double leftEnc = relmastLeftEncoder.getPosition() * 360 / gearRatioLeft;
          double rightEnc = relmastRightEncoder.getPosition() * 360 / gearRatioRight;
          SmartDashboard.putNumber("Mast Left Encoder", leftEnc);
          SmartDashboard.putNumber("Mast Right Encoder", rightEnc);
          SmartDashboard.putNumber("Mast Abs Encoder", absMastLeftEncoder.getAbsolutePosition());
        });
    // return this.startEnd(
    //     () -> {
    //       SmartDashboard.putBoolean("Mast.moving", true);
    //       setMastSpeed(controllerSpeed);
    //     },
    //     () -> {
    //       SmartDashboard.putBoolean("Mast.moving", false);
    //       stop();
    //     });
  }

  // public double getMastPosition(){

  // }

  // public Command mastTargetAmp(){
  //     return this.runOnce();
  // }

  // END OF Mast Class
}
