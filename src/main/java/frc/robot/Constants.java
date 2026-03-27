// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

  public static final String limeLight = "limelight";
  public static final String c4Name = "ShootCamL";
  public static final String c3Name = "ShootCamBR";
  public static final String c2Name = "Intake Cam";

  /* Back of the robot Shoot Cam L */
  public static Transform3d c4 =
      new Transform3d(-0.241, -0.285, 0.279, new Rotation3d(0, -2.845, -3.107));

  /* Shoot Cam BR */
  public static Transform3d c3 =
      new Transform3d(0.318, 0.286, 0.267, new Rotation3d(0, -0.279, -0.052));

  /* Front Intake Cam */
  public static Transform3d c2 =
      new Transform3d(0.229, -0.254, 0.343, new Rotation3d(0, 1.5707, -3.14159));

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

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
