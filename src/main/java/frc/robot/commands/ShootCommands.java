// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.structures.ShootIntake;

/** Add your docs here. */
public class ShootCommands {

  private ShootCommands() {}
  //TODO: ensure voltages set to correct values
  public static Command rampFlyWheel(ShootIntake shootIntake, double volts) {
    return Commands.run(() -> shootIntake.rampFlyWheel(volts), shootIntake)
        .handleInterrupt(() -> shootIntake.stopFlyWheel());
  }

  public static Command feed(ShootIntake shootIntake) {
    return Commands.run(() -> shootIntake.feedShooter(6), shootIntake)
        .handleInterrupt(() -> shootIntake.stopFeeder());
  }

  public static Command intake(ShootIntake shootIntake) {
    return Commands.race(rampFlyWheel(shootIntake, -6)
        .handleInterrupt(() -> shootIntake.stopFlyWheel())
        .alongWith(rampFlyWheel(shootIntake, 6))
                .handleInterrupt(() -> shootIntake.stopFeeder()));
  }

  public static Command extake(ShootIntake shootIntake) {
     return Commands.race(rampFlyWheel(shootIntake, -6)
        .handleInterrupt(() -> shootIntake.stopFlyWheel())
        .alongWith(rampFlyWheel(shootIntake, -6))
                .handleInterrupt(() -> shootIntake.stopFeeder()));
  }

  public static Command pass(ShootIntake shootIntake) {
    return Commands.race(rampFlyWheel(shootIntake, 6)
        .handleInterrupt(() -> shootIntake.stopFlyWheel())
        .alongWith(rampFlyWheel(shootIntake, 6))
                .handleInterrupt(() -> shootIntake.stopFeeder()));
  }

  /* Eventually implement distance calibrations  */ 

  public static Command launchSequence(ShootIntake shootIntake) {
     return Commands.race(
        rampFlyWheel(shootIntake, 0.5)
            .withTimeout(.5)
            .andThen(rampFlyWheel(shootIntake, 1))
                .alongWith(feed(shootIntake)));
    }
  
  public static Command autolaunchSequence(ShootIntake shootIntake) {
    return Commands.race(
        rampFlyWheel(shootIntake, 0.5)
            .withTimeout(1)
            .andThen(rampFlyWheel(shootIntake, 1).withTimeout(5))
                .alongWith(feed(shootIntake)));
  }
}
