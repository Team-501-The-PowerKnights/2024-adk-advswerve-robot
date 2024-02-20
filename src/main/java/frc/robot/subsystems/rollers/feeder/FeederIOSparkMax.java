// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.feeder;

import com.revrobotics.CANSparkMax;

public class FeederIOSparkMax implements FeederIO {
  // Hardware Setup
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  public FeederIOSparkMax() {

    leftMotor = new CANSparkMax(22, CANSparkMax.MotorType.kBrushless);
    rightMotor = new CANSparkMax(23, CANSparkMax.MotorType.kBrushless);

    // Default
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    // Limits
    leftMotor.setSmartCurrentLimit(20);
    rightMotor.setSmartCurrentLimit(20);
    leftMotor.enableVoltageCompensation(12.0);
    rightMotor.enableVoltageCompensation(12.0);

    // Disable brake mode
    leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {

    inputs.leftAppliedVolts = leftMotor.getAppliedOutput();
    inputs.leftOutputCurrent = leftMotor.getOutputCurrent();
    inputs.leftTempCelsius = leftMotor.getMotorTemperature();

    inputs.rightAppliedVolts = rightMotor.getAppliedOutput();
    inputs.rightOutputCurrent = rightMotor.getOutputCurrent();
    inputs.rightTempCelsius = rightMotor.getMotorTemperature();
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
