package frc.robot.subsystems.rollers.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.GenericRollerSystemIOSim;

public class FeederIOSim extends GenericRollerSystemIOSim implements FeederIO {
  private static final DCMotor motorModel = DCMotor.getNEO(1);
  private static final double reduction = 15 / 1.0;
  private static final double moi = 0.001;

  public FeederIOSim() {
    super(motorModel, reduction, moi);
  }
}
