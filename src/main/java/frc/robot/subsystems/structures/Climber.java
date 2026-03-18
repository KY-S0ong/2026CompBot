// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private SparkMax c1Motor = new SparkMax(40, MotorType.kBrushless);
  private double gearRatio = 16.0;

  private double targetPosition = 100; // Example target position, adjust as needed

  @SuppressWarnings("deprecation")
  public Climber() {
    c1Motor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putSmartDahsboard();
  }

  public String timeToClimb() {
    double currentPosition = c1Motor.getEncoder().getPosition() / gearRatio;
    double velocity = c1Motor.getEncoder().getVelocity();

    if (velocity == 0) {
      return "Stationary";
    }
    return Double.toString((targetPosition - currentPosition) / velocity);
  }

  public void positionClimb() {
    c1Motor.setVoltage(5);
    // Add logic to stop the motor when the target position is reached
    if (c1Motor.getEncoder().getPosition() * gearRatio >= 100) { // Example target position
      c1Motor.setVoltage(0);
    }
  }

  public void positionDeclimb() {
    c1Motor.setVoltage(-4);
    // Add logic to stop the motor when the target position is reached
    if (c1Motor.getEncoder().getPosition() * gearRatio <= 0) { // Example target position
      c1Motor.setVoltage(0);
    }
  }

  public void setClimberVolts(double volts) {
    c1Motor.setVoltage(volts);
  }

  public void manualClimb(double set) {
    c1Motor.set(set);
  }

  public void putSmartDahsboard() {
    SmartDashboard.putNumber("Climber Position", c1Motor.getEncoder().getPosition() / gearRatio);
    SmartDashboard.putString("Time to Climb", timeToClimb());
  }
}
