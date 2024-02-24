// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  private static RobotType robotType = RobotType.SIMBOT;
  public static final Mode currentMode = Mode.SIM;
  public static final boolean tuningMode = true;

  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    DEVBOT,
    COMPBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  public static final class IntakeConstants {
    // SPARK MAX CAN IDs
    public static final int kIntakeFront = 20;
    public static final int kIntakeRear = 21;
    // Speed Controller Speeds
    public static final double kIntakeSpeed = 1;
    public static final int kIntakeCurrentLimit = 80;
  }

  public static final class FeederConstants {
    // SPARK MAX CAN IDs
    public static final int kFeederRight = 22;
    // Speed Controller Speeds
    public static final double kFeederSpeed = 1;
    public static final int kFeederCurrentLimit = 80;
  }

  public static final class IncrementerConstants {
    // SPARK MAX CAN IDs
    public static final int kIncrementerLeft = 30;
    public static final int kIncrementerRight = 31;
    // Speed Controller Speeds
    public static final double kIncrementerSpeed = 1;
    public static final int kIncrementerCurrentLimit = 80;
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
    public static final int kMastCurrentLimit = 80;
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType.toString());
      System.exit(1);
    }
  }
}
