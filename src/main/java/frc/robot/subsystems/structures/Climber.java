// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private SparkMax c1Motor = new SparkMax(40, MotorType.kBrushless);
  private SparkMax c2Motor = new SparkMax(41, MotorType.kBrushless);
  
  public Climber() {
    c1Motor.setInverted(false);
    c2Motor.setInverted(true);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
