// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

  // private TalonFX flyWheelShooter = new TalonFX(51);
  private TalonFX flyWheel2 = new TalonFX(53); // left
  private TalonFX flyWheel1 = new TalonFX(51); // right
  // private MotorOutputConfigs flyWheelShooterConfiguration = new MotorOutputConfigs();
  // private MotorOutputConfigs feederConfiguration = new MotorOutputConfigs();
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private TalonFXConfiguration config2 = new TalonFXConfiguration();

  private double gearRatio = 1.0;
  private double height = 1.91;
  private double fuelMassLBS = 0.448;
  private double fuelMassKG = .2177243;

  public Flywheel() {

    // flyWheelShooterConfiguration.withNeutralMode(NeutralModeValue.Coast);
    // flyWheelShooter.getConfigurator().apply(flyWheelShooterConfiguration);
    // 1.8
    double ks = 0.65;
    double kv = 0.125; // 0.12
    double kp = 1.25; // 0.05
    config.Slot0.kP = kp;
    config.Slot0.kV = kv;
    config.Slot0.kS = ks;

    config.CurrentLimits.StatorCurrentLimit = 145;
    config.CurrentLimits.SupplyCurrentLimit = 80;

    // flyWheelShooter.getConfigurator().apply(config);

    config2.Slot0.kP = kp;
    config2.Slot0.kV = kv;
    config2.Slot0.kS = ks;

    config2.CurrentLimits.StatorCurrentLimit = 145;
    config2.CurrentLimits.SupplyCurrentLimit = 80;

    // config2.Slot0.kD = .0001;

    flyWheel2.getConfigurator().apply(config2);

    flyWheel1.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    SmartDashboardUpdate();
  }

  public void rampFlyWheel(double volts) {
    flyWheel1.setVoltage(-volts);
    flyWheel2.setVoltage(volts);
  }

  public void stopFlyWheel() {
    flyWheel1.set(0);
    flyWheel2.set(0);
  }

  private double getTargetVelocity() {
    double distance = SmartDashboard.getNumber("Shot Distance", 3.0);
    double velocity = (8.55 * distance) + 29.5;
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

  public void smartFlyWheel(double addedVel) {
    double flywheelVelocity = getTargetVelocity();

    flyWheel1.setControl(new VelocityVoltage(-flywheelVelocity - addedVel).withEnableFOC(true));
    flyWheel2.setControl(new VelocityVoltage(flywheelVelocity + addedVel).withEnableFOC(true));
  }

  public void setFlyVelocity(double velocity) {

    flyWheel1.setControl(new VelocityVoltage(-velocity).withEnableFOC(true));
    flyWheel2.setControl(new VelocityVoltage(velocity).withEnableFOC(true));
  }

  public boolean isAtTargetVelocity() {
    double currentVelocity = gearRatio * flyWheel2.getVelocity().getValueAsDouble();
    double targetVelocity = getTargetVelocity();
    double tolerance = 1.0; // RPS tolerance
    return Math.abs(currentVelocity - targetVelocity) < tolerance;
  }

  private void SmartDashboardUpdate() {
    SmartDashboard.putNumber("FlyWheel RPS", flyWheel2.getVelocity().getValueAsDouble());

    /*BigDecimal voltageRounded = new BigDecimal(getTargetVolts());
    voltageRounded = voltageRounded.setScale(3, BigDecimal.ROUND_HALF_UP);
    SmartDashboard.putNumber("Desired Shot Voltage", voltageRounded.doubleValue());*/

    SmartDashboard.putNumber("Desired RPS", getTargetVelocity());
  }
}
