// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.structures.Feeder;
import frc.robot.subsystems.structures.Flywheel;

/** Add your docs here. */
public class ShootCommands {

  private ShootCommands() {}

  public static Command rampFlyWheel(Flywheel flyWheel, double volts) {
    return Commands.run(() -> flyWheel.rampFlyWheel(volts), flyWheel)
        .handleInterrupt(() -> flyWheel.stopFlyWheel());
  }

  public static Command smartFlyWheel(Flywheel flyWheel) {
    return Commands.run(() -> flyWheel.smartFlyWheel(), flyWheel)
        .handleInterrupt(() -> flyWheel.stopFlyWheel());
  }

  public static Command feedFly(Feeder feeder) {
    return Commands.run(() -> feeder.feedShooter(-3), feeder)
        .handleInterrupt(() -> feeder.stopFeeder());
  }

  public static ParallelRaceGroup intake(Flywheel flyWheel, Feeder feeder) {
    return new ParallelRaceGroup(
        rampFlyWheel(flyWheel, 7).handleInterrupt(() -> flyWheel.stopFlyWheel()),
        Commands.run(() -> feeder.feedShooter(7), feeder)
            .handleInterrupt(() -> feeder.stopFeeder()));
  }

  public static ParallelRaceGroup extake(Flywheel flyWheel, Feeder feeder) {
    return new ParallelRaceGroup(
        rampFlyWheel(flyWheel, -7).handleInterrupt(() -> flyWheel.stopFlyWheel()),
        Commands.run(() -> feeder.feedShooter(-7), feeder)
            .handleInterrupt(() -> feeder.stopFeeder()));
  }

  /* Eventually implement distance calibrations */

  public static SequentialCommandGroup launchSequence(Flywheel flyWheel, Feeder feeder) {
    return new SequentialCommandGroup(
        // rampFlyWheel(flyWheel, flyWheel.getTargetVolts()).withTimeout(.35),
        smartFlyWheel(flyWheel).withTimeout(0.35),
        feedFly(feeder).alongWith(smartFlyWheel(flyWheel)));
  }

  public static SequentialCommandGroup voltLaunch(Flywheel flyWheel, Feeder feeder, double volts) {
    return new SequentialCommandGroup(
        rampFlyWheel(flyWheel, volts).withTimeout(.35),
        feedFly(feeder).alongWith(rampFlyWheel(flyWheel, volts)));
  }

  public static SequentialCommandGroup autolaunchSequence(Flywheel flyWheel, Feeder feeder) {
    return new SequentialCommandGroup(
        smartFlyWheel(flyWheel).withTimeout(0.35),
        feedFly(feeder).alongWith(smartFlyWheel(flyWheel)));
  }
}
