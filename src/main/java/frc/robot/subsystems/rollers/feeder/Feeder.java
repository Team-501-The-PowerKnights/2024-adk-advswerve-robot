// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.feeder;

import frc.robot.subsystems.rollers.GenericRollerSystem;
import frc.robot.subsystems.rollers.GenericRollerSystem.VoltageGoal;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
public class Feeder extends GenericRollerSystem<Feeder.Goal> {

  private final FeederIO io;

  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageGoal {
    IDLE(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Feeder/FloorIntakingVoltage", 8.0)),
    BACKSTOPPING(new LoggedTunableNumber("Feeder/BackstoppingVoltage", -4.0)),
    SHOOTING(new LoggedTunableNumber("Feeder/Shooting", 8.0)),
    EJECTING(new LoggedTunableNumber("Feeder/EjectingVoltage", -6.0));

    private final DoubleSupplier voltageSupplier;
  }

  public Feeder(FeederIO io) {
    super("Feeder", io);
    this.io = io;
  }

  private Goal goal = Goal.IDLE;

  public Goal getGoal() {
    return goal;
  }

  public void setGoal(Goal goal) {
    goal = this.goal;
  }
}
