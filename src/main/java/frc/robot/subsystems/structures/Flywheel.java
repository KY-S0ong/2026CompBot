// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import java.math.BigDecimal;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

  private TalonFX intakeShooter = new TalonFX(51);
  //private TalonFX feeder = new TalonFX(52);
  private MotorOutputConfigs intakeShooterConfiguration = new MotorOutputConfigs();
  //private MotorOutputConfigs feederConfiguration = new MotorOutputConfigs();

  private double gearRatio = 1.0;
  private double height = 1.91;
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
    double theta = Units.degreesToRadians(75);

    double velocity =
        (distance / Math.cos(theta)) * Math.sqrt(g / 2 * (Math.tan(theta) * distance - height));
    return velocity;
  }

  public double getTargetVolts() {

    // double targetVel = getTargetVelocity();
    // double rps = targetVel / ((Math.PI * 2) * 0.1016);

    //double distance = SmartDashboard.getNumber("Shot Distance", 2.5);
    double distance = SmartDashboard.getNumber("Moving Shot Distance", 2.5);
    // double targetVoltage = 21.86 / (1 + Math.pow(Math.E, -0.219 * (distance - 5.45))) *
    // Math.sqrt(height/1.905);
    double targetVoltage = (3.1 * Math.sqrt(distance) + 2.55) * Math.sqrt(height / 1.9);
    return Math.min(targetVoltage, 12);

    /*
     * if (rps <= 0) return 10.0;
     *
     * // Constants - you should tune these!
     * double kS = 3.0; // Volts to overcome friction
     * double kV = 0.05; // Volts per meter/second (example value)
     *
     * double voltage = kS + (kV * rps);
     *
     * // Cap the voltage to the battery limit (12V)
     * return Math.min(voltage, 12.0);
     */
  }

  public void smartFlyWheel() {}

  public void TESTsetFlyWheelVelocity() {
    intakeShooter.setControl(new VelocityVoltage(getTargetVelocity() / ((Math.PI * 2) * 4)));
  }

  @SuppressWarnings("deprecation")
  private void SmartDashboardUpdate() {
    BigDecimal velocityRounded =
        new BigDecimal(
            gearRatio * intakeShooter.getVelocity().getValueAsDouble() * ((Math.PI * 2) * 0.1016));
    velocityRounded = velocityRounded.setScale(4, BigDecimal.ROUND_HALF_UP);
    
    SmartDashboard.putNumber(
        "FlyWheel RPM", velocityRounded.doubleValue());
    
    BigDecimal voltageRounded = new BigDecimal(getTargetVolts());
    voltageRounded = voltageRounded.setScale(4, BigDecimal.ROUND_HALF_UP);
    SmartDashboard.putNumber("Desired Shot Voltage", voltageRounded.doubleValue());
    //SmartDashboard.putNumber("Desired Vel", getTargetVelocity());
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
