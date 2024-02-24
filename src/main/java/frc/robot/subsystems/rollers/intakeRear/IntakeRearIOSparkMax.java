// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.intakeRear;

import frc.robot.subsystems.rollers.GenericRollerSystemIOSparkMax;

public class IntakeRearIOSparkMax extends GenericRollerSystemIOSparkMax implements IntakeRearIO {
  private static final int id = 21;
  private static final int currentLimitAmps = 20;
  private static final boolean invert = false;
  private static final boolean brake = false;
  private static final double reduction = 5 / 1;

  public IntakeRearIOSparkMax() {

    super(id, currentLimitAmps, invert, brake, reduction);
  }
}
