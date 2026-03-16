// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
// Modified by FRC 3958
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.FieldConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.05;
  // All angle-based PID constants now use DriveConstants values via rotationController
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier isFieldRelative) {

    if (isFieldRelative.getAsBoolean() == true) {
      return Commands.run(
          () -> {
            // Get linear velocity
            Translation2d linearVelocity =
                getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

            // Apply rotation deadband
            double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

            // Square rotation value for more precise control
            omega = Math.copySign(omega * omega, omega);

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds =
                new ChassisSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec());
            boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    isFlipped
                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                        : drive.getRotation()));
          },
          drive);
    } else {
      return Commands.run(
          () -> {
            // Get linear velocity
            Translation2d linearVelocity =
                getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

            // Apply rotation deadband
            double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

            // Square rotation value for more precise control
            omega = Math.copySign(omega * omega, omega);

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds =
                new ChassisSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec());
            drive.runVelocity(speeds);
          },
          drive);
    }
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command joystickHubDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Pose2d> pose2dSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed

              double desiredRad = drive.getShotAngle(pose2dSupplier);
              double omega =
                  angleController.calculate(drive.getRotation().getRadians(), desiredRad);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  /**
   * Autonomous command that rotates the robot to point at the hub for shooting. This command should
   * be used after following a path to your shoot position. It will rotate the robot to face the hub
   * center while holding position.
   *
   * @param drive The drive subsystem
   * @return A command that rotates the robot to point at the hub
   */
  public static Command pointAtHubForShoot(Drive drive) {
    // Create PID controller for rotation
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Calculate angle to hub from current position
              double desiredAngle = drive.getShotAngle(() -> drive.getPose());

              // Calculate angular speed to rotate toward hub
              double omega =
                  angleController.calculate(drive.getRotation().getRadians(), desiredAngle);

              // Only apply rotation, no linear movement
              ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, omega);
              drive.runVelocity(speeds);
            },
            drive)
        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Autonomous command that ensures the robot is at a safe distance from the hub before shooting.
   * If the robot is too close, it will back up to the minimum shooting distance while maintaining
   * its heading toward the hub. Uses trapezoidal motion profiling for smooth acceleration.
   *
   * <p>This command automatically ends when the robot reaches the minimum distance.
   *
   * @param drive The drive subsystem
   * @return A command that backs up to minimum distance if needed
   */
  public static Command ensureMinimumShootingDistance(Drive drive) {
    // Create PID controller for rotation to keep facing hub
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Create PID controller for distance backup motion with trapezoidal profiling
    ProfiledPIDController distanceController =
        new ProfiledPIDController(
            DriveConstants.Y_CENTERING_KP, // kP for distance
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                DriveConstants.backupMaxVelocity, DriveConstants.backupMaxAcceleration));

    // Construct command
    return Commands.run(
            () -> {
              // Get current distance to hub
              double currentDistance = drive.getShotDistance().in(edu.wpi.first.units.Units.Meter);

              // Calculate desired distance (setpoint)
              double targetDistance = DriveConstants.minimumShootingDistance;

              // Calculate backward velocity using distance PID controller
              // Negative velocity means backward
              double backwardVelocity =
                  -distanceController.calculate(currentDistance, targetDistance);

              // Calculate angle to hub to maintain heading while backing up
              double desiredAngle = drive.getShotAngle(() -> drive.getPose());
              double omega =
                  angleController.calculate(drive.getRotation().getRadians(), desiredAngle);

              // Move backward while maintaining heading
              ChassisSpeeds speeds = new ChassisSpeeds(backwardVelocity, 0.0, omega);
              drive.runVelocity(speeds);
            },
            drive)
        // Reset PID controllers when command starts
        .beforeStarting(
            () -> {
              angleController.reset(drive.getRotation().getRadians());
              distanceController.reset(drive.getShotDistance().in(edu.wpi.first.units.Units.Meter));
            })
        // End condition: stop moving when we reach minimum distance (within 5cm tolerance)
        .until(
            () ->
                drive.getShotDistance().in(edu.wpi.first.units.Units.Meter)
                    >= (DriveConstants.minimumShootingDistance - 0.05));
  }

  /**
   * Vision-based command that centers the robot on the Y-axis under a trench opening while allowing
   * driver control for X-axis movement (forward/back through trench). The robot will automatically
   * maintain its Y position centered in the trench opening, but the driver can move
   * forward/backward along the trench using joystick input.
   *
   * @param drive The drive subsystem
   * @param isLeftTrench True for left trench, false for right trench
   * @param xSupplier Driver input for X-axis movement (forward/back)
   * @return A command that centers on Y-axis while allowing driver X control
   */
  private static Command centerUnderTrench(
      Drive drive, boolean isLeftTrench, DoubleSupplier xSupplier) {
    // Create PID controller for Y-axis centering only
    ProfiledPIDController yController =
        new ProfiledPIDController(
            DriveConstants.Y_CENTERING_KP, // kP for Y-axis centering
            DriveConstants.Y_CENTERING_KI,
            DriveConstants.Y_CENTERING_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.backupMaxVelocity, DriveConstants.backupMaxAcceleration));

    // Create PID controller for rotation to maintain heading
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Get target Y position (center of trench opening)
    double targetY =
        isLeftTrench
            ? (FieldConstants.LeftTrench.openingTopLeft.getY()
                    + FieldConstants.LeftTrench.openingTopRight.getY())
                / 2.0
            : (FieldConstants.RightTrench.openingTopLeft.getY()
                    + FieldConstants.RightTrench.openingTopRight.getY())
                / 2.0;

    return Commands.run(
            () -> {
              Pose2d currentPose = drive.getPose();

              // Get driver input for X-axis (forward/back through trench)
              double driverVx = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
              driverVx = Math.copySign(driverVx * driverVx, driverVx); // Square for control

              // Calculate Y velocity to center in trench
              double vy = yController.calculate(currentPose.getY(), targetY);

              // Maintain heading at the snapped cardinal angle (set at start)
              double omega =
                  angleController.calculate(drive.getRotation().getRadians(), angleController.getSetpoint());

              // Apply driver X input scaled to max speed
              double vx = driverVx * drive.getMaxLinearSpeedMetersPerSec();

              // Send velocities in field-relative frame
              ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        // Snap to nearest cardinal angle and reset controllers when command starts
        .beforeStarting(
            () -> {
              Pose2d startPose = drive.getPose();
              yController.reset(startPose.getY());
              
              // Snap to nearest cardinal angle (0, π/2, π, 3π/2)
              double currentAngleRad = drive.getRotation().getRadians();
              currentAngleRad = ((currentAngleRad % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
              
              double[] cardinalAngles = {0, Math.PI / 2, Math.PI, 3 * Math.PI / 2};
              double minDifference = Double.MAX_VALUE;
              double desiredAngle = 0;
              
              for (double cardinal : cardinalAngles) {
                double difference = Math.abs(currentAngleRad - cardinal);
                if (difference < minDifference) {
                  minDifference = difference;
                  desiredAngle = cardinal;
                }
              }
              
              angleController.reset(desiredAngle);
            })
        // End condition: within 5cm of target Y position
        .until(
            () -> {
              Pose2d currentPose = drive.getPose();
              double currentTargetY =
                  isLeftTrench
                      ? (FieldConstants.LeftTrench.openingTopLeft.getY()
                              + FieldConstants.LeftTrench.openingTopRight.getY())
                          / 2.0
                      : (FieldConstants.RightTrench.openingTopLeft.getY()
                              + FieldConstants.RightTrench.openingTopRight.getY())
                          / 2.0;
              return Math.abs(currentPose.getY() - currentTargetY) < 0.05;
            });
  }

  /**
   * Vision-based command that drives to center the robot underneath the nearest trench opening.
   * Automatically determines which trench (left or right) is closer to the current robot position
   * and drives to center under that trench.
   *
   * <p>This is useful for autonomous routines where you don't know which side the robot will start
   * on, or for dynamic positioning during a match. The robot will auto-center on the Y-axis while
   * allowing driver control over X-axis movement through the trench.
   *
   * @param drive The drive subsystem
   * @param xSupplier A supplier for driver X-axis input (forward/back movement in the trench)
   * @return A command that drives to center under the nearest trench
   */
  public static Command centerUnderNearestTrench(
      Drive drive, java.util.function.DoubleSupplier xSupplier) {
    return Commands.select(
        java.util.Map.ofEntries(
            java.util.Map.entry(true, centerUnderTrench(drive, true, xSupplier)),
            java.util.Map.entry(false, centerUnderTrench(drive, false, xSupplier))),
        () -> {
          // Determine which trench is closer
          Pose2d currentPose = drive.getPose();
          double distanceToLeftTrench =
              Math.abs(
                  currentPose.getY()
                      - (FieldConstants.LeftTrench.openingTopLeft.getY()
                              + FieldConstants.LeftTrench.openingTopRight.getY())
                          / 2.0);
          double distanceToRightTrench =
              Math.abs(
                  currentPose.getY()
                      - (FieldConstants.RightTrench.openingTopLeft.getY()
                              + FieldConstants.RightTrench.openingTopRight.getY())
                          / 2.0);

          // Return true if left trench is closer, false for right
          return distanceToLeftTrench <= distanceToRightTrench;
        });
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
