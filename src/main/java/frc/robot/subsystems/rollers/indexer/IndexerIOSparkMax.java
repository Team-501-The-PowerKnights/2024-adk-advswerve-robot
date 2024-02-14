// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.indexer;

import frc.robot.subsystems.rollers.GenericRollerSystemIOSparkMax;

public class IndexerIOSparkMax extends GenericRollerSystemIOSparkMax implements IndexerIO {
  private static final double reduction = (18.0 / 12.0); // todo
  private static final int id = 42;
  private static final int currentLimitAmps = 40;
  private static final boolean inverted = false;

  public IndexerIOSparkMax() {
    super(id, currentLimitAmps, inverted, true, reduction);
  }
}
