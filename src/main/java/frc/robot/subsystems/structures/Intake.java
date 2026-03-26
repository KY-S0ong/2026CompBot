// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  
  private TalonFX intake = new TalonFX(51);
  private MotorOutputConfigs config = new MotorOutputConfigs();

  public Intake() {
    config.withNeutralMode(NeutralModeValue.Coast);
    intake.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double volts) {
    intake.setVoltage(volts);
  }
  
  public void stopIntake() {
    intake.set(0);
  }
}
