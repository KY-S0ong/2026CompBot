// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final Pose3d redHubPose =
      new Pose3d(
          Units.Inches.of(468.56),
          Units.Inches.of(158.32),
          Units.Inches.of(72.0),
          new Rotation3d());
  public static final Pose3d blueHubPose =
      new Pose3d(
          Units.Inches.of(152.56),
          Units.Inches.of(158.32),
          Units.Inches.of(72.0),
          new Rotation3d());

  public static final Pose3d redFerryPose = new Pose3d(14.3, 4.02, 0, Rotation3d.kZero);
  public static final Pose3d blueFerryPose = new Pose3d(2.1, 4.02, 0, Rotation3d.kZero);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
