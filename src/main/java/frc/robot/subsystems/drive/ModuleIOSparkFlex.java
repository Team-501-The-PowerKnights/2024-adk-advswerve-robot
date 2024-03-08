// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkFlex implements ModuleIO {
  private static final double SWERVE_PINION_TEETH = 14;
  private static final double DRIVE_GEAR_RATIO = (45.0 * 22) / (SWERVE_PINION_TEETH * 15);
  private static final double TURN_GEAR_RATIO = (9424.0 / 203); // Rev Steering

  private static final double ABS_OFFSET[] = {270.0, 180.0, 0.0, 90.0};

  private final CANSparkFlex driveSparkFlex;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isDriveMotorInverted = false;
  private final boolean isTurnMotorInverted = false;
  private final Rotation2d absoluteEncoderOffset;

  // STU
  private final int m_index;

  public ModuleIOSparkFlex(int index) {
    m_index = index;
    switch (index) {
      case 0: // FL
        driveSparkFlex = new CANSparkFlex(1, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        // MUST BE CALIBRATED
        absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(273.95));
        break;
      case 1: // FR
        driveSparkFlex = new CANSparkFlex(5, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(6, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        // MUST BE CALIBRATED
        absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(179.49));
        break;
      case 2: // BL
        driveSparkFlex = new CANSparkFlex(3, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        // MUST BE CALIBRATED
        absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(0.0));
        break;
      case 3: // BR
        driveSparkFlex = new CANSparkFlex(7, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(8, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        // MUST BE CALIBRATED
        absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(88.85)); // 88.85
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkFlex.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkFlex.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkFlex.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    driveSparkFlex.setInverted(isDriveMotorInverted);
    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkFlex.setSmartCurrentLimit(40);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkFlex.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(15);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    // double position =
    //     Rotation2d.fromRotations(turnAbsoluteEncoder.getPosition())
    //         .minus(absoluteEncoderOffset)
    //         .getRotations();
    // turnRelativeEncoder.setPosition(position * TURN_GEAR_RATIO);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkFlex.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkFlex.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);

    driveSparkFlex.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkFlex.getAppliedOutput() * driveSparkFlex.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkFlex.getOutputCurrent()};

    /* Original */
    // inputs.turnAbsolutePosition =
    //     new Rotation2d(
    //             turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 *
    // Math.PI)
    //         .minus(absoluteEncoderOffset);

    /* For Calibration of Offsets */
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber(
        "turnCalibration[" + m_index + "]", inputs.turnAbsolutePosition.getDegrees());

    /* Our Version */
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsoluteEncoder.getPosition()).minus(absoluteEncoderOffset);
    SmartDashboard.putNumber(
        "turnAbsolutePosition[" + m_index + "]", inputs.turnAbsolutePosition.getDegrees());
    SmartDashboard.putBoolean(
        "turnAligned[" + m_index + "]", (Math.abs(inputs.turnAbsolutePosition.getDegrees()) < 0.3));
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    SmartDashboard.putNumber(
        "turnRelativePosition[" + m_index + "]", inputs.turnPosition.getDegrees());
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkFlex.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
