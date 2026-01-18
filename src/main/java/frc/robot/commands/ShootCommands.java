// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.structures.ShootIntake;

/** Add your docs here. */
public class ShootCommands {

  private ShootCommands() {}

  private Command rampFlyWheel(ShootIntake shootIntake) {
    return Commands.run(() -> shootIntake.rampFlyWheel(0.5), shootIntake);
  }
  private Command stopFlyWheel(ShootIntake shootIntake){
    return Commands.run(()-> shootIntake.stopFlyWheel(), shootIntake);
  }

  public Command launchSequence(ShootIntake shootIntake) {
    return new SequentialCommandGroup(
        rampFlyWheel(shootIntake).withTimeout(7),
        stopFlyWheel(shootIntake).withTimeout(5)
    );
  }
}
