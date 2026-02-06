// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        Math.sqrt(
            (-4.9 * Math.pow(distance, 2))
                / (Math.pow(Math.cos(shooterHoodRad), 2) * (-height - Math.tan(shooterHoodRad))));

    return velocity;
  }

  public double getShotVolts() {

    double distance = SmartDashboard.getNumber("Shot Distance", fuelMass);

    double velocity = getShotVelocity(distance);

    double time = (velocity + Math.sqrt(Math.pow(velocity, 2) * (-2 * 9.8 * height))) / 9.8;

    double power = fuelMass * Math.pow(velocity, 2) / (2 * time);

    return power / 40;
  }

  private void SmartDashboardUpdate() {
    SmartDashboard.putNumber(
        "FlyWheel RPM", gearRatio * intakeShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Desired Shot Voltage", getShotVolts());
  }
}
