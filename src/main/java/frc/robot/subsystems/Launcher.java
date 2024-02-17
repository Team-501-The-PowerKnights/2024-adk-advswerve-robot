package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  CANSparkMax launcherLeft;
  CANSparkMax launcherRight;
  double launcherSpeed;

  public Launcher() {
    launcherLeft = new CANSparkMax(kLauncherLeft, MotorType.kBrushless);
    launcherRight = new CANSparkMax(kLauncherRight, MotorType.kBrushless);
    launcherLeft.setSmartCurrentLimit(kLauncherCurrentLimit);
    launcherRight.setSmartCurrentLimit(kLauncherCurrentLimit);
    launcherRight.setInverted(true);
    launcherRight.follow(launcherLeft);
    launcherSpeed = kLauncherSpeed;
    System.out.println("Launcher Constructed!!");
  }
  // Sets the speed of the lead motor
  public void setLauncherSpeed(double speed) {
    launcherLeft.set(speed);
  }
  // Sets the speed of the lead motor to 0
  public void stop() {
    launcherLeft.set(0);
  }
  // Use this command to pull a note off the floor
  public Command runLauncher() {
    return this.startEnd(
        () -> {
          setLauncherSpeed(kLauncherSpeed);
        },
        () -> {
          stop();
        });
  }
  // Use this command to "eject" a note back onto the floor
  public Command reverseLauncher() {
    return this.startEnd(
        () -> {
          setLauncherSpeed(kLauncherSpeed * -0.5);
        },
        () -> {
          stop();
        });
  }

  // END OF Lancher Class
}
