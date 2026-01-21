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
  // fix parrell commands
  public static Command rampFlyWheel(ShootIntake shootIntake, double volts) {
    return Commands.run(() -> shootIntake.rampFlyWheel(volts), shootIntake)
        .handleInterrupt(() -> shootIntake.stopFlyWheel());
  }

  public static Command feed(ShootIntake shootIntake) {
    return Commands.run(() -> shootIntake.feedShooter(6), shootIntake)
        .handleInterrupt(() -> shootIntake.stopFeeder());
  }

  public static Command intake(ShootIntake shootIntake) {
    return Commands.run(() -> shootIntake.rampFlyWheel(-3), shootIntake)
            .handleInterrupt(() -> shootIntake.stopFlyWheel()).alongWith(
        Commands.run(() -> shootIntake.feedShooter(6), shootIntake)
            .handleInterrupt(() -> shootIntake.stopFeeder()));
  }

  public static Command extake(ShootIntake shootIntake) {
    return new ParallelCommandGroup(
        Commands.run(() -> shootIntake.rampFlyWheel(-3), shootIntake)
            .handleInterrupt(() -> shootIntake.stopFlyWheel()),
        Commands.run(() -> shootIntake.feedShooter(-6), shootIntake)
            .handleInterrupt(() -> shootIntake.stopFeeder()));
  }

  public static Command pass(ShootIntake shootIntake) {
    return Commands.run(() -> shootIntake.rampFlyWheel(6), shootIntake)
            .handleInterrupt(() -> shootIntake.stopFlyWheel()).alongWith(
                Commands.run(() -> shootIntake.feedShooter(-12), shootIntake)
            .handleInterrupt(() -> shootIntake.stopFeeder()));
  }

  public static Command launchSequence(ShootIntake shootIntake) {
    return new SequentialCommandGroup(
        rampFlyWheel(shootIntake, 12).withTimeout(1),
        feed(shootIntake)
            .handleInterrupt(() -> shootIntake.stopFeeder())
            .alongWith(
                rampFlyWheel(shootIntake, 12).handleInterrupt(() -> shootIntake.stopFlyWheel())));
  }

  public static Command autolaunchSequence(ShootIntake shootIntake) {
    return new SequentialCommandGroup(
        rampFlyWheel(shootIntake, 12).withTimeout(.5),
        feed(shootIntake)
            .withTimeout(5.5)
            .alongWith(rampFlyWheel(shootIntake, 12).withTimeout(5.5)));
  }
}
