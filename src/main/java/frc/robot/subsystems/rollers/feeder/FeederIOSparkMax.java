// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.feeder;

import frc.robot.subsystems.rollers.GenericRollerSystemIOSparkMax;

public class FeederIOSparkMax extends GenericRollerSystemIOSparkMax implements FeederIO {
  private static final int id = 3;
  private static final int currentLimitAmps = 40;
  private static final boolean invert = false;
  private static final boolean brake = false;
  private static final double reduction = 18.0 / 12.0;

  public FeederIOSparkMax() {
    super(id, currentLimitAmps, invert, brake, reduction);
  }
}
