package frc.robot.subsystems.rollers.intakeRear;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.GenericRollerSystemIOSim;

public class IntakeRearIOSim extends GenericRollerSystemIOSim implements IntakeRearIO {

  private static final DCMotor motorModel = DCMotor.getNEO(1);
  private static final double reduction = (18.0 / 12.0);
  private static final double moi = 0.001;

  public IntakeRearIOSim() {
    super(motorModel, reduction, moi);
  }
}
