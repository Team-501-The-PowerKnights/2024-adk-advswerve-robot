package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TopFeederSensor extends SubsystemBase {
  DigitalInput feederSensor;
  DigitalGlitchFilter feederSensorFilter;

  public TopFeederSensor() {

    feederSensor = new DigitalInput(0);
    /*
    if (RobotBase.isReal()) {
      // Note Detection Filter for Real Robots
      feederSensorFilter = new DigitalGlitchFilter();
      feederSensorFilter.add(feederSensor);
      feederSensorFilter.setPeriodNanoSeconds(5000000); // 50ms constant to filter glitch
    }
    */
  }

  // so other subsystems can read sensor
  public boolean get() {
    return !feederSensor.get();
  }

  // so other subsystems can read sensor
  public boolean getInv() {
    return feederSensor.get();
  }

  // Runs in Periodic
  @Override
  public void periodic() {
    Logger.recordOutput("Feeder/NotePOS", get());
  }
}
