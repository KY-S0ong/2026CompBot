// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.structures.Feeder;
import frc.robot.subsystems.structures.Flywheel;

/** Add your docs here. */
public class ShootCommands {

  private ShootCommands() {}
  // TODO: ensure voltages set to correct values
  public static Command rampFlyWheel(Flywheel shootIntake, double volts) {
    return Commands.run(() -> shootIntake.rampFlyWheel(volts), shootIntake)
        .handleInterrupt(() -> shootIntake.stopFlyWheel());
  }

  public static Command feed(Feeder feeder) {
    return Commands.run(() -> feeder.feedShooter(6), feeder)
        .handleInterrupt(() -> feeder.stopFeeder());
  }

  public static Command intake(Flywheel flyWheel, Feeder feeder) {
    return rampFlyWheel(flyWheel, -6)
        .handleInterrupt(() -> flyWheel.stopFlyWheel())
        .alongWith(rampFlyWheel(flyWheel, 6))
        .handleInterrupt(() -> feeder.stopFeeder());
  }

  public static Command extake(Flywheel flyWheel, Feeder feeder) {
    return rampFlyWheel(flyWheel, -6)
        .handleInterrupt(() -> flyWheel.stopFlyWheel())
        .alongWith(rampFlyWheel(flyWheel, -6))
        .handleInterrupt(() -> feeder.stopFeeder());
  }

  public static Command pass(Flywheel flyWheel, Feeder feeder) {
    return rampFlyWheel(flyWheel, 6)
        .handleInterrupt(() -> flyWheel.stopFlyWheel())
        .alongWith(rampFlyWheel(flyWheel, 6))
        .handleInterrupt(() -> feeder.stopFeeder());
  }

  /* Eventually implement distance calibrations  */

  public static Command launchSequence(Flywheel flyWheel, Feeder feeder) {
    return rampFlyWheel(flyWheel, 0.5)
        .withTimeout(.5)
        .andThen(rampFlyWheel(flyWheel, 1))
        .alongWith(feed(feeder));
  }

  public static Command autolaunchSequence(Flywheel flyWheel, Feeder feeder) {
    return rampFlyWheel(flyWheel, 0.5)
        .withTimeout(1)
        .andThen(rampFlyWheel(flyWheel, 1).withTimeout(5))
        .alongWith(feed(feeder));
  }
}
