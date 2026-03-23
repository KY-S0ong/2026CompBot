package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import java.util.Optional;

public class DriveConstants {
  public static final double maxSpeed =
      TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
  public static final double maxAngularRate =
      Units.RotationsPerSecond.of(0.75)
          .in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  public static final Distance shooterSideOffset = Units.Inches.of(6.0);

  public static final double minimumShootingDistance = 1.0; // meters - minimum safe distance to hub
  // Use fractions of max drive speed/acceleration for smooth backup motion
  public static final double backupMaxVelocity = maxSpeed * 0.3; // 30% of max speed
  public static final double backupMaxAcceleration =
      (maxSpeed * 0.3) / 0.5; // reach 30% speed in 0.5s

  public static final Transform2d shooterTransform =
      new Transform2d(Units.Inches.of(0.0), shooterSideOffset, new Rotation2d());

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

  public static final Angle epsilonAngleToGoal = Degrees.of(1.0);

  public static final Pose3d getHubPose() {
    Pose3d pose =
        DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;

    return pose;
  }

  public static final Pose3d getFerryPose() {
    Pose3d pose =
        DriverStation.getAlliance().equals(Optional.of(Alliance.Red))
            ? redFerryPose
            : blueFerryPose;
    return pose;
  }

  public static final PIDController rotationController = getRotationController();

  private static final PIDController getRotationController() {
    PIDController controller = new PIDController(2.0, 0.0, 0.0);
    controller.enableContinuousInput(-Math.PI, Math.PI);
    return controller;
  }

  // PID constants for autonomous motion profiling
  public static final double Y_CENTERING_KP = 2.0;
  public static final double Y_CENTERING_KI = 0.0;
  public static final double Y_CENTERING_KD = 0.0;

  // Angle control PID constants for profiled controllers
  public static final double ANGLE_KP = 6.05;
  public static final double ANGLE_KI = 0.0;
  public static final double ANGLE_KD = 0.4;
  public static final double ANGLE_MAX_VELOCITY = 8.0; // rad/s
  public static final double ANGLE_MAX_ACCELERATION = 20.0; // rad/s^2
}
