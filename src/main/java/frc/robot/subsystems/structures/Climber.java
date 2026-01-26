// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private SparkMax c1Motor = new SparkMax(40, MotorType.kBrushless);
  private double gearRatio = 16.0;

  public Climber() {
    c1Motor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double timeToClimb() {
    double currentPosition = c1Motor.getEncoder().getPosition() * gearRatio;
    double velocity = c1Motor.getEncoder().getVelocity();

    return (currentPosition / velocity);
  }

  public void setClimberVolts(double volts) {
    c1Motor.setVoltage(volts);
  }
}
