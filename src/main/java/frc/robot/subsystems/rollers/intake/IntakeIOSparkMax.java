// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.intake;

import frc.robot.subsystems.rollers.GenericRollerSystemIOSparkMax;

public class IntakeIOSparkMax extends GenericRollerSystemIOSparkMax implements IntakeIO {
  private static final int id = 40;
  private static final int currentLimitAmps = 40;
  private static final boolean invert = false;
  private static final boolean brake = false;
  private static final double reduction = 18.0 / 12.0; // TODO: Gear Ratio Intake Sub

  public IntakeIOSparkMax() {

    super(id, currentLimitAmps, invert, brake, reduction);
  }
}
