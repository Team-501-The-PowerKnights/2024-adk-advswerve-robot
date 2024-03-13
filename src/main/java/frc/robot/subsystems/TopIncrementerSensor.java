package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TopIncrementerSensor extends SubsystemBase {
  DigitalInput incrementerSensor;
  DigitalGlitchFilter incrementerSensorFilter;

  public TopIncrementerSensor() {

    incrementerSensor = new DigitalInput(1);

    if (RobotBase.isReal()) {
      // Note Detection Filter for Real Robots
      incrementerSensorFilter = new DigitalGlitchFilter();
      incrementerSensorFilter.add(incrementerSensor);
      incrementerSensorFilter.setPeriodNanoSeconds(50000000); // 50ms constant to filter glitch
    }
  }

  // so other subsystems can read sensor
  public boolean get() {
    return !incrementerSensor.get();
  }

  // Runs in with Periodic
  @Override
  public void periodic() {
    Logger.recordOutput("Increment/NotePOS", get());
  }
}
