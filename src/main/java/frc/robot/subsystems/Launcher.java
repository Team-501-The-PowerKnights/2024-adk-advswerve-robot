package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  // CANSparkMax launcherLeft;
  // CANSparkMax launcherRight;
  TalonFX launcherLeft;
  TalonFX launcherRight;
  double launcherSpeed;

  public Launcher() {
    launcherLeft = new TalonFX(kLauncherLeft);
    launcherRight = new TalonFX(kLauncherRight);

    // launcherLeft.setSmartCurrentLimit(kLauncherCurrentLimit);
    // launcherRight.setSmartCurrentLimit(kLauncherCurrentLimit);
    launcherRight.setInverted(true);
    //    launcherRight.setControl(launcherLeft);
    launcherSpeed = kLauncherSpeed;
    System.out.println("Launcher Constructed!!");
  }
  // Sets the speed of the lead motor
  public void setLauncherSpeed(double speed) {
    launcherLeft.set(speed);
    launcherRight.set(speed);
  }
  // Sets the speed of the lead motor to 0
  public void stop() {
    launcherLeft.set(0);
    launcherRight.set(0);
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
