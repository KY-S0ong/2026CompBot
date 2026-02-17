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

  public static Command feedFly(Feeder feeder) {
    return Commands.run(() -> feeder.smartFeed(-1), feeder)
        .handleInterrupt(() -> feeder.stopFeeder());
  }

  public static ParallelRaceGroup intake(Flywheel flyWheel, Feeder feeder) {
    return new ParallelRaceGroup(
        rampFlyWheel(flyWheel, -6).handleInterrupt(() -> flyWheel.stopFlyWheel()),
        Commands.run(() -> feeder.feedShooter(2), feeder)
            .handleInterrupt(() -> feeder.stopFeeder()));
  }

  /* Eventually implement distance calibrations */

  public static SequentialCommandGroup launchSequence(Flywheel flyWheel, Feeder feeder) {
    return new SequentialCommandGroup(
        rampFlyWheel(flyWheel, flyWheel.getTargetVolts()).withTimeout(.25),
        feedFly(feeder).alongWith(rampFlyWheel(flyWheel, flyWheel.getTargetVolts())));
  }

  // REMOVE Later
  public static Command testShot(Flywheel flyWheel) {
    return Commands.run(() -> flyWheel.rampFlyWheel(flyWheel.getTargetVolts()), flyWheel)
        .handleInterrupt(() -> flyWheel.stopFlyWheel());
  }

  public static SequentialCommandGroup autolaunchSequence(Flywheel flyWheel, Feeder feeder) {
    return new SequentialCommandGroup(
        rampFlyWheel(flyWheel, 0.5).withTimeout(0.35),
        rampFlyWheel(flyWheel, 1).withTimeout(5),
        feedFly(feeder));
  }
}
