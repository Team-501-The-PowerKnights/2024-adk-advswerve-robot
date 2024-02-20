// Copyright (c) 2024 FRC 6328, 501
// http://github.com/Powerknights
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.feeder;

import frc.robot.subsystems.rollers.GenericRollerSystemIO;

public interface FeederIO extends GenericRollerSystemIO {

  public class FeederIOInputs {
    public double leftAppliedVolts = 0.0;
    public double leftOutputCurrent = 0.0;
    public double leftTempCelsius = 0.0;

    public double rightAppliedVolts = 0.0;
    public double rightOutputCurrent = 0.0;
    public double rightTempCelsius = 0.0;
  }

  /** Update inputs */
  default void updateInputs(FeederIOInputs inputs) {}

  /** Run both motors at voltage */
  default void runVolts(double leftVolts, double rightVolts) {}

  /** Stop both motors */
  default void stop() {}
}
