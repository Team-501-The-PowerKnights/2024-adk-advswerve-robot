// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.intakeFront;

import frc.robot.subsystems.rollers.GenericRollerSystemIOSparkMax;

public class IntakeFrontIOSparkMax extends GenericRollerSystemIOSparkMax implements IntakeFrontIO {
  private static final int id = 20;
  private static final int currentLimitAmps = 20;
  private static final boolean invert = true;
  private static final boolean brake = false;
  private static final double reduction = 5 / 1;

  public IntakeFrontIOSparkMax() {

    super(id, currentLimitAmps, invert, brake, reduction);
  }
}
