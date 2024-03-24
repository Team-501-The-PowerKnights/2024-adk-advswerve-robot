package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimbLimitSensors extends SubsystemBase {
  DigitalInput LeftClimbSensor;
  DigitalInput RightClimbSensor;
  DigitalGlitchFilter incrementerSensorFilter;

  public ClimbLimitSensors() {

    LeftClimbSensor = new DigitalInput(2);
    RightClimbSensor = new DigitalInput(4);
    /*
    if (RobotBase.isReal()) {
      // Note Detection Filter for Real Robots
      incrementerSensorFilter = new DigitalGlitchFilter();
      incrementerSensorFilter.add(incrementerSensor);
      incrementerSensorFilter.setPeriodNanoSeconds(5000000); // 50ms constant to filter glitch
    }
    */
  }

  // so other subsystems can read sensor
  public boolean get() {
    return (!LeftClimbSensor.get() || !RightClimbSensor.get());
  }

  // Runs in with Periodic
  @Override
  public void periodic() {
    Logger.recordOutput("Climber/Left Sensor", LeftClimbSensor.get());
    Logger.recordOutput("Climber/Right Sensor", RightClimbSensor.get());
  }
}
