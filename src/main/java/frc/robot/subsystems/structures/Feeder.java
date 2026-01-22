// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

  private TalonFX feeder = new TalonFX(31);
  private MotorOutputConfigs feederConfiguration = new MotorOutputConfigs();

  public Feeder() {
    feederConfiguration.withNeutralMode(NeutralModeValue.Coast);
    feeder.getConfigurator().apply(feederConfiguration);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void feedShooter(double volts) {
    feeder.setVoltage(volts);
  }

  public void stopFeeder() {
    feeder.set(0);
  }
}
