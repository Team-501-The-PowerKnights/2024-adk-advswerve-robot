package frc.robot.subsystems;

import static frc.robot.Constants.MastConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mast extends SubsystemBase {
  CANSparkMax mastLeft;
  CANSparkMax mastRight;
  double mastSpeed;
  RelativeEncoder relmastLeft;
  RelativeEncoder relmastRight;

  public Mast() {
    mastLeft = new CANSparkMax(kMastLeft, MotorType.kBrushless);
    mastLeft.restoreFactoryDefaults();
    mastRight = new CANSparkMax(kMastRight, MotorType.kBrushless);
    mastRight.restoreFactoryDefaults();

    mastLeft.setSmartCurrentLimit(kMastCurrentLimit);
    mastRight.setSmartCurrentLimit(kMastCurrentLimit);
    mastRight.setInverted(true);
    mastRight.follow(mastLeft);
    relmastLeft = mastLeft.getEncoder();
    relmastRight = mastRight.getEncoder();

    // mastSpeed = kMastSpeed;//Speed will be controlled by axis from remote
    System.out.println("Mast Constructed!!");
  }

  // Sets the speed of the lead motor
  public void setMastSpeed(double speed) {
    mastLeft.set(speed);
  }

  // Sets the speed of the lead motor to 0
  public void stop() {
    mastLeft.set(0);
  }

  public Command mastUpDown(double controllerSpeed) {
    return this.startEnd(
        () -> {
          if (controllerSpeed >= -1 && controllerSpeed <= 1) {
            setMastSpeed(controllerSpeed);
          }
        },
        () -> {
          stop();
        });
  }

  // public double getMastPosition(){

  // }

  // public Command mastTargetAmp(){
  //     return this.runOnce();
  // }

  // END OF Mast Class
}
