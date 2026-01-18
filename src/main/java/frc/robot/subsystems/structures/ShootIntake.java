// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShootIntake extends SubsystemBase {

  private TalonFX intakeShooter = new TalonFX(31);
  private TalonFX feeder = new TalonFX(32);
  private MotorOutputConfigs intakeShooterConfiguration = new MotorOutputConfigs();
  private MotorOutputConfigs feederConfiguration = new MotorOutputConfigs();

  private double gearRatio = 1.0;

  public ShootIntake() {
    // intakeShooterConfiguration.Inverted = InvertedValue.CounterClockwise_Positive;
    // feederConfiguration.Inverted = InvertedValue.CounterClockwise_Positive;

    intakeShooterConfiguration.withNeutralMode(NeutralModeValue.Coast);
    feederConfiguration.withNeutralMode(NeutralModeValue.Coast);

    intakeShooter.getConfigurator().apply(intakeShooterConfiguration);
    feeder.getConfigurator().apply(feederConfiguration);
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

  private void SmartDashboardUpdate() {
    SmartDashboard.putNumber(
        "FlyWheel RPM", gearRatio * intakeShooter.getVelocity().getValueAsDouble());
  }
}
