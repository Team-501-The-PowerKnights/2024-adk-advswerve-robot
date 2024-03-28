// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Robot currentRobot = Robot.REAL;

  public static enum Robot {
    /** Running on suitcase robot. */
    SUITCASE,

    /** Running on proto robot. (drive chassis) */
    PROTO,

    /** Running on real robot. (all subsystems) */
    REAL
  }

  public static final class IntakeConstants {
    // SPARK MAX CAN IDs
    public static final int kIntakeFront = 20;
    public static final int kIntakeRear = 21;
    // Speed Controller Speeds
    public static final double kIntakeSpeed = 1;
    public static final int kIntakeCurrentLimit = 20;
  }

  public static final class FeederConstants {
    // SPARK MAX CAN IDs
    public static final int kFeederRight = 22;
    // Speed Controller Speeds
    public static final double kFeederSpeed = 1;
    public static final int kFeederCurrentLimit = 40;
  }

  public static final class IncrementerConstants {
    // SPARK MAX CAN IDs
    public static final int kIncrementerLeft = 30;
    public static final int kIncrementerRight = 31;
    // Speed Controller Speeds
    public static final double kIncrementerSpeed = 1;
    public static final int kIncrementerCurrentLimit = 20;
  }

  public static final class LauncherConstants {
    // SPARK MAX CAN IDs
    public static final int kLauncherLeft = 32;
    public static final int kLauncherRight = 33;
    // Speed Controller Speeds
    public static final double kLauncherSpeed = 1;
    public static final int kLauncherCurrentLimit = 80;
  }

  public static final class MastConstants {
    // SPARK MAX CAN IDs
    public static final int kMastLeft = 40;
    public static final int kMastRight = 41;
    // Speed Controller Speeds
    public static final double kMastSpeed = 1;
    public static final int kMastCurrentLimit = 30;
  }

  public static final class ClimberConstants {
    // SPARK MAX CAN IDs
    public static final int kClimber = 50;

    // Speed Controller Speeds
    public static final double kClimberSpeed = 1;
    public static final int kClimberCurrentLimit = 20;
  }
}
