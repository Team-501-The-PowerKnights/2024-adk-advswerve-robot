package frc.robot.subsystems;

import static frc.robot.Constants.MastConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Mast extends SubsystemBase {
  CANSparkMax mastLeft;
  CANSparkMax mastRight;
  double mastSpeed;
  RelativeEncoder relmastLeft;
  RelativeEncoder relmastRight;
  CommandXboxController operPad;

  public Mast() {
    mastLeft = new CANSparkMax(kMastLeft, MotorType.kBrushless);
    mastLeft.restoreFactoryDefaults();
    mastRight = new CANSparkMax(kMastRight, MotorType.kBrushless);
    mastRight.restoreFactoryDefaults();

    mastLeft.setSmartCurrentLimit(kMastCurrentLimit);
    mastRight.setSmartCurrentLimit(kMastCurrentLimit);

    mastLeft.enableVoltageCompensation(12.0);
    mastRight.enableVoltageCompensation(12.0);

    mastRight.setInverted(true);
    // mastRight.follow(mastLeft);
    relmastLeft = mastLeft.getEncoder();
    relmastRight = mastRight.getEncoder();

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
    mastLeft.set(0);
    mastRight.set(0);
  }

  public Command mastUpDown(double controllerSpeed, CommandXboxController operator) {
    return this.run(
        () -> {
          SmartDashboard.putBoolean("Mast.moving", true);
          System.out.println("running mastUpDown = " + operator.getLeftY());
          setMastSpeed(operator.getLeftY());
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
