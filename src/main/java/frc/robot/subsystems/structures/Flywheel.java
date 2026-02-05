// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import static edu.wpi.first.units.Units.Meter;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
public class Flywheel extends SubsystemBase {

  private TalonFX intakeShooter = new TalonFX(30);
  private MotorOutputConfigs intakeShooterConfiguration = new MotorOutputConfigs();

  private double gearRatio = 1.0;
  private double height = 100;
  private double fuelMass = 0.448;

  public Flywheel() {
    // intakeShooterConfiguration.Inverted =
    // InvertedValue.CounterClockwise_Positive;

    intakeShooterConfiguration.withNeutralMode(NeutralModeValue.Coast);

    intakeShooter.getConfigurator().apply(intakeShooterConfiguration);
  }

  @Override
  public void periodic() {
    SmartDashboardUpdate();
  }

  public void rampFlyWheel(double volts) {
    intakeShooter.setVoltage(volts);
  }

  public void stopFlyWheel() {
    intakeShooter.set(0);
  }

  private double getShotVelocity(double distance) {
    double shooterHoodRad = 0.99483;
    double velocity =

        Math.sqrt((-4.9 * Math.pow(distance, 2)) /
            (Math.pow(Math.cos(shooterHoodRad), 2) * (-height - Math.tan(shooterHoodRad))));

    return velocity;
  }

  public Distance getShotDistance(Translation2d targetPose, Supplier<Pose2d> poseSupplier) {
    Pose2d drivePose = poseSupplier.get();
    double centerToTargetMeters = drivePose.getTranslation().getDistance(targetPose);
    double centerToShooterMeters = DriveConstants.shooterSideOffset.in(Units.Meters);
    double shooterToTargetMeters =
        Math.sqrt(Math.pow(centerToTargetMeters, 2.0) - Math.pow(centerToShooterMeters, 2.0));
    return Units.Meters.of(shooterToTargetMeters);
  }

  public Distance getShotDistance(Supplier<Pose2d> poseSupplier) {
    return getShotDistance(DriveConstants.getHubPose().toPose2d().getTranslation(), poseSupplier);
  }

  public double getShotVolts(Supplier<Pose2d> poseSupplier) {
    double distance = getShotDistance(poseSupplier).in(Meter);

    double velocity = getShotVelocity(distance);

    double time = (velocity + Math.sqrt(Math.pow(velocity, 2) * (-2 * 9.8 * height))) / 9.8;

    double power = fuelMass * Math.pow(velocity, 2) / (2 * time);

    return power/40;
  }

  private void SmartDashboardUpdate() {
    SmartDashboard.putNumber(
        "FlyWheel RPM", gearRatio * intakeShooter.getVelocity().getValueAsDouble());
  }
}
