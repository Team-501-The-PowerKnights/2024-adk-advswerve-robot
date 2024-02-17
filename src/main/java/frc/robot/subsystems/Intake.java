package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  CANSparkMax intakeFront;
  CANSparkMax intakeRear;
  double intakeSpeed;

  public Intake() {
    intakeFront = new CANSparkMax(kIntakeFront, MotorType.kBrushless);
    intakeFront.restoreFactoryDefaults();
    intakeRear = new CANSparkMax(kIntakeRear, MotorType.kBrushless);
    intakeRear.restoreFactoryDefaults();

    intakeFront.setSmartCurrentLimit(kIntakeCurrentLimit);
    intakeRear.setSmartCurrentLimit(kIntakeCurrentLimit);
    intakeRear.setInverted(true);
    intakeRear.follow(intakeFront);
    intakeSpeed = kIntakeSpeed;
    System.out.println("Intake Constructed!!");
  }
  // Sets the speed of the lead motor
  public void setIntakeSpeed(double speed) {
    intakeFront.set(speed);
  }
  // Sets the speed of the lead motor to 0
  public void stop() {
    intakeFront.set(0);
  }
  // Use this command to pull a note off the floor
  public Command runIntake() {
    return this.startEnd(
        () -> {
          setIntakeSpeed(kIntakeSpeed);
        },
        () -> {
          stop();
        });
  }
  // Use this command to "eject" a note back onto the floor
  public Command reverseIntake() {
    return this.startEnd(
        () -> {
          setIntakeSpeed(kIntakeSpeed * -0.5);
        },
        () -> {
          stop();
        });
  }

  // END OF INTAKE CLASS
}
