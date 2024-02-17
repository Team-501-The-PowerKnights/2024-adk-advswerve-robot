package frc.robot.subsystems;

import static frc.robot.Constants.FeederConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

  // CANSparkMax feederLeft;
  CANSparkMax feederRight;
  double feederSpeed;

  public Feeder() {
    //        feederLeft = new CANSparkMax(kFeederLeft, MotorType.kBrushless);
    feederRight = new CANSparkMax(kFeederRight, MotorType.kBrushless);
    // feederLeft.setSmartCurrentLimit(kFeederCurrentLimit);
    feederRight.setSmartCurrentLimit(kFeederCurrentLimit);
    feederRight.setInverted(true);
    // feederRight.follow(feederLeft);
    feederSpeed = kFeederSpeed;
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
  // Use this command to pull a note off the floor
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
