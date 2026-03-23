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

  // Shooter motors: Kraken X60 brushless motors controlled via TalonFX
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
    // PID gains for velocity control
    config.Slot0.kP = 4.5;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.001;
    config.Slot0.kS = 0.001;
    // kV = 12V / (6000 RPM / 60) = 12V / 100 RPS = 0.12 V/RPS (Kraken X60 feedforward)
    config.Slot0.kV = 0.12;

    intakeShooter.getConfigurator().apply(config);

    config2.Slot0.kP = 4.5;
    config2.Slot0.kI = 0.0;
    config2.Slot0.kD = .001;
    config2.Slot0.kS = .001;
    // kV = 12V / (6000 RPM / 60) = 12V / 100 RPS = 0.12 V/RPS (Kraken X60 feedforward)
    config2.Slot0.kV = 0.12;

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
    double velocity = (0.088 * Math.pow(distance, 2)) + (3.65 * distance) + 17.67586;
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
    double flywheelVelocity = getTargetVelocity();
    flywheelVelocity *= 1.9;

    ;
    double topRatio = 44.0 / 56.0;
    double topMotorVel = flywheelVelocity * topRatio;

    intakeShooter.setControl(new VelocityVoltage(topMotorVel));
    intake2.setControl(new VelocityVoltage(flywheelVelocity));
  }

  public void setFlyVelocity(double velocity) {
    velocity *= 1.9;
    intakeShooter.setControl(new VelocityVoltage(velocity));
    intake2.setControl(new VelocityVoltage(velocity));
  }

  /**
   * Checks if the flywheel has reached the target velocity within tolerance. Used to determine when
   * the hopper can begin feeding.
   *
   * @return true if actual velocity is within 2 RPS of target velocity
   */
  public boolean isAtTargetVelocity() {
    double currentVelocity =
        gearRatio * intakeShooter.getVelocity().getValueAsDouble() * ((Math.PI * 2) * 0.1016);
    double targetVelocity = getTargetVelocity();
    double tolerance = 2.0; // RPS tolerance
    return Math.abs(currentVelocity - targetVelocity) < tolerance;
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
