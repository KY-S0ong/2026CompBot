// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.math.BigDecimal;

public class Flywheel extends SubsystemBase {

  private TalonFX intakeShooter = new TalonFX(51);
  private TalonFX intake2 = new TalonFX(53);
  // private TalonFX feeder = new TalonFX(52);
  // private MotorOutputConfigs intakeShooterConfiguration = new MotorOutputConfigs();
  // private MotorOutputConfigs feederConfiguration = new MotorOutputConfigs();
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private TalonFXConfiguration config2 = new TalonFXConfiguration();

  private double gearRatio = 1.0;
  private double height = 1.91;
  private double fuelMassLBS = 0.448;
  private double fuelMassKG = .2177243;

  public Flywheel() {
    // intakeShooterConfiguration.Inverted =
    // InvertedValue.CounterClockwise_Positive;

    // intakeShooterConfiguration.withNeutralMode(NeutralModeValue.Coast);
    // intakeShooter.getConfigurator().apply(intakeShooterConfiguration);
    config.Slot0.kP = 1.55;
    config.Slot0.kD = 0.001;
    config.Slot0.kS = 0.001;

    intakeShooter.getConfigurator().apply(config);

    config2.Slot0.kP = 1.45;
    config2.Slot0.kD = .001;
    config2.Slot0.kS = .001;

    intake2.getConfigurator().apply(config2);
  }

  @Override
  public void periodic() {
    SmartDashboardUpdate();
  }

  public void rampFlyWheel(double volts) {
    intakeShooter.setVoltage(volts);
    intake2.setVoltage(volts);
  }

  public void stopFlyWheel() {
    intakeShooter.set(0);
    intake2.set(0);
  }

  private double getTargetVelocity() {
    double distance = SmartDashboard.getNumber("Shot Distance", 2.0);
    double velocity = (0.088 * Math.pow(distance, 2)) + (3.855 * distance) + 16.3686;
    return velocity;
  }

  public double getTargetVolts() {

    // double targetVel = getTargetVelocity();
    // double rps = targetVel / ((Math.PI * 2) * 0.1016);

    double distance = SmartDashboard.getNumber("Shot Distance", 2.0);
    // double distance = SmartDashboard.getNumber("Moving Shot Distance", 2.5);
    // double targetVoltage = 21.86 / (1 + Math.pow(Math.E, -0.219 * (distance - 5.45))) *
    // Math.sqrt(height/1.905);
    double targetVoltage = (3.15 * Math.sqrt(distance) + 2.55) * Math.sqrt(height / 1.9);
    return Math.min(targetVoltage, 12);
  }

  public void smartFlyWheel() {
    double velocity = getTargetVelocity();
    velocity *= 1.80;
    intakeShooter.setControl(new VelocityVoltage(velocity));
    intake2.setControl(new VelocityVoltage(velocity));
  }

  @SuppressWarnings("deprecation")
  private void SmartDashboardUpdate() {
    BigDecimal velocityRounded =
        new BigDecimal(
            gearRatio * intakeShooter.getVelocity().getValueAsDouble() * ((Math.PI * 2) * 0.1016));
    velocityRounded = velocityRounded.setScale(3, BigDecimal.ROUND_HALF_UP);

    SmartDashboard.putNumber("FlyWheel RPS", velocityRounded.doubleValue());

    /*BigDecimal voltageRounded = new BigDecimal(getTargetVolts());
    voltageRounded = voltageRounded.setScale(3, BigDecimal.ROUND_HALF_UP);
    SmartDashboard.putNumber("Desired Shot Voltage", voltageRounded.doubleValue());*/

    BigDecimal desiredRPSRounded = new BigDecimal(getTargetVelocity());
    desiredRPSRounded = desiredRPSRounded.setScale(3, BigDecimal.ROUND_HALF_UP);
    SmartDashboard.putNumber("Desired RPS", desiredRPSRounded.doubleValue());
  }
}
