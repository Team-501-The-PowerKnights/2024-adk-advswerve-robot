package frc.robot.subsystems;

import static frc.robot.Constants.IncrementerConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Incrementer extends SubsystemBase {
  CANSparkMax incrementerLeft;
  CANSparkMax incrementerRight;
  double incrementerSpeed;

  public Incrementer() {
    incrementerLeft = new CANSparkMax(kIncrementerLeft, MotorType.kBrushless);
    incrementerLeft.restoreFactoryDefaults();
    incrementerRight = new CANSparkMax(kIncrementerRight, MotorType.kBrushless);
    incrementerLeft.restoreFactoryDefaults();

    incrementerLeft.setSmartCurrentLimit(kIncrementerCurrentLimit);
    incrementerRight.setSmartCurrentLimit(kIncrementerCurrentLimit);

    incrementerLeft.setInverted(false);
    incrementerRight.setInverted(false);
    // incrementerRight.follow(incrementerLeft);
    incrementerSpeed = kIncrementerSpeed;
    System.out.println("Incrementer Constructed!!");
  }
  // Sets the speed of the lead motor
  public void setIncrementerSpeed(double speed) {
    try {
      Thread.sleep(2000);
    } catch (Exception ex) {
    }
    incrementerLeft.set(-speed);
    incrementerRight.set(speed);
  }
  // Sets the speed of the lead motor to 0
  public void stop() {
    incrementerLeft.set(0);
    incrementerRight.set(0);
  }
  // Use this command to pull a note off the floor
  public Command runIncrementer() {
    return this.startEnd(
        () -> {
          setIncrementerSpeed(kIncrementerSpeed);
        },
        () -> {
          stop();
        });
  }
  // Use this command to "eject" a note back onto the floor
  public Command reverseIncrementer() {
    return this.startEnd(
        () -> {
          setIncrementerSpeed(kIncrementerSpeed * -1.0);
        },
        () -> {
          stop();
        });
  }
}
