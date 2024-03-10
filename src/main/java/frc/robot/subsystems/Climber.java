package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  TalonFX climberFX;
  double climberSpeed;

  public Climber() {
    climberFX = new TalonFX(kClimber);
    // climberFX.setInverted(true);
    climberSpeed = kClimberSpeed;

    System.out.println("Climber Constructed!!");
  }

  // Sets the speed of the lead motor
  public void setClimberSpeed(double speed) {
    climberFX.set(-speed);
  }
  // Sets the speed of the lead motor to 0
  public void stop() {
    climberFX.set(0);
  }
  // Use this command to pull a note off the floor
  public Command runClimber() {
    return this.startEnd(
        () -> {
          setClimberSpeed(kClimberSpeed);
        },
        () -> {
          stop();
        });
  }
  // Use this command to "eject" a note back onto the floor
  public Command reverseClimber() {
    return this.startEnd(
        () -> {
          setClimberSpeed(kClimberSpeed * -1.0);
        },
        () -> {
          stop();
        });
  }

  // END OF Climber Class
}
