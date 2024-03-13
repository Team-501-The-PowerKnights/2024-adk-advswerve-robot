package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TopIncrementerSensor extends SubsystemBase {
  DigitalInput feederSensor;
  DigitalGlitchFilter feederSensorFilter;

  public TopIncrementerSensor() {
  

    // Note Detection Sensor
    feederSensor = new DigitalInput(1);
    feederSensorFilter = new DigitalGlitchFilter();
    feederSensorFilter.add(feederSensor);
    feederSensorFilter.setPeriodNanoSeconds(50000000); // 50ms constant to filter glitch
  }

  // so other subsystems can read sensor
  public boolean get() {
    return feederSensor.get();
  }



}
