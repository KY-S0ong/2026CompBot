// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

  private TalonFX intakeShooter = new TalonFX(30);
  private MotorOutputConfigs intakeShooterConfiguration = new MotorOutputConfigs();

  private double gearRatio = 1.0;
  private double height = 100;
  private double fuelMassLBS = 0.448;
  private double fuelMassKG = .2177243;

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

  private double getTargetVelocity() {
    double distance = SmartDashboard.getNumber("Shot Distance", 2.0);
    double g = 9.806;
    double theta = 0.99483; // Launch angle in radians

    double velocity =
        (distance / Math.cos(theta))
            * Math.sqrt((g / 2) * 1 / (Math.tan(theta) * distance - height));
    return velocity;
  }

  public double getTargetVolts() {

    double targetVel = getTargetVelocity();

    if (targetVel <= 0) return 10.0;

    // Constants - you should tune these!
    double kS = 0.4; // Volts to overcome friction
    double kV = 0.5; // Volts per meter/second (example value)

    double voltage = kS + (kV * targetVel);

    // Cap the voltage to the battery limit (12V)
    return Math.min(voltage, 12.0);
  }

  public void TESTsetFlyWheelVelocity() {
    intakeShooter.setControl(new VelocityVoltage(getTargetVelocity() / ((Math.PI * 2) * 4)));
  }

  private void SmartDashboardUpdate() {
    SmartDashboard.putNumber(
        "FlyWheel RPM", gearRatio * intakeShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Desired Shot Voltage", getTargetVolts());
  }

  /*
   * private double AminTargetVelocity(){
   * double distance = SmartDashboard.getNumber("Shot Distance", 2.0 );
   *
   * double g = 9.806;
   * double theta = 0.99483; // Launch angle in radians
   * double deltaHeight = 2.64;
   *
   * double velocity = Math.sqrt(
   * (g * Math.pow(distance, 2)) /
   * (2 * Math.pow(Math.cos(theta), 2) * (distance * Math.tan(theta) -
   * deltaHeight)));
   *
   * return velocity;
   * }
   */
}
