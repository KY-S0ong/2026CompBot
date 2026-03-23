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

  public static Command stopAll(Flywheel flywheel, Feeder feeder) {
    return Commands.run(() -> flywheel.stopFlyWheel(), flywheel);
  }

  public static Command rampFlyWheel(Flywheel flyWheel, double volts) {
    return Commands.run(() -> flyWheel.rampFlyWheel(volts), flyWheel)
        .handleInterrupt(() -> flyWheel.stopFlyWheel());
  }

  public static Command smartFlyWheel(Flywheel flyWheel) {
    return Commands.run(() -> flyWheel.smartFlyWheel(), flyWheel)
        .handleInterrupt(() -> flyWheel.stopFlyWheel());
  }

  public static Command feedFly(Feeder feeder) {
    return Commands.run(() -> feeder.feedShooter(-6), feeder)
        .handleInterrupt(() -> feeder.stopFeeder());
  }

  public static Command smartFeedFly(Feeder feeder, Flywheel flyWheel) {
    return Commands.run(() -> feeder.smartFeed(-3, flyWheel), feeder)
        .handleInterrupt(() -> feeder.stopFeeder());
  }

  public static ParallelRaceGroup intake(Flywheel flyWheel, Feeder feeder) {
    return new ParallelRaceGroup(
        rampFlyWheel(flyWheel, 6).handleInterrupt(() -> flyWheel.stopFlyWheel()),
        Commands.run(() -> feeder.feedShooter(6), feeder)
            .handleInterrupt(() -> feeder.stopFeeder()));
  }

  public static ParallelRaceGroup extake(Flywheel flyWheel, Feeder feeder) {
    return new ParallelRaceGroup(
        rampFlyWheel(flyWheel, -8).handleInterrupt(() -> flyWheel.stopFlyWheel()),
        Commands.run(() -> feeder.feedShooter(-8), feeder)
            .handleInterrupt(() -> feeder.stopFeeder()));
  }

  public static ParallelRaceGroup unJam(Flywheel flyWheel, Feeder feeder) {
    return new ParallelRaceGroup(
        rampFlyWheel(flyWheel, 12).handleInterrupt(() -> flyWheel.stopFlyWheel()),
        Commands.run(() -> feeder.feedShooter(12), feeder)
            .handleInterrupt(() -> feeder.stopFeeder()));
  }

  /* Eventually implement distance calibrations */

  public static SequentialCommandGroup launchSequence(Flywheel flyWheel, Feeder feeder) {
    return new SequentialCommandGroup(
        // Ramp flywheel until it reaches target velocity, with 1 second timeout
        smartFlyWheel(flyWheel).until(() -> flyWheel.isAtTargetVelocity()).withTimeout(1.0),
        // Once ready, feed the shooter while maintaining flywheel speed
        feedFly(feeder).alongWith(smartFlyWheel(flyWheel)));
  }

  public static SequentialCommandGroup overrideLaunch(
      Flywheel flyWheel, Feeder feeder, double velocity) {
    return new SequentialCommandGroup(
        Commands.run(() -> flyWheel.setFlyVelocity(velocity), flyWheel).withTimeout(0.7),
        feedFly(feeder).alongWith(Commands.run(() -> flyWheel.setFlyVelocity(velocity), flyWheel)));
  }

  /*public static SequentialCommandGroup voltLaunch(Flywheel flyWheel, Feeder feeder, double volts) {
    return new SequentialCommandGroup(
        rampFlyWheel(flyWheel, volts).withTimeout(.35),
        feedFly(feeder).alongWith(rampFlyWheel(flyWheel, volts)));
  }*/

  public static SequentialCommandGroup autolaunchSequence(Flywheel flyWheel, Feeder feeder) {
    return new SequentialCommandGroup(
        // Ramp flywheel until it reaches target velocity, with 1 second timeout
        smartFlyWheel(flyWheel).until(() -> flyWheel.isAtTargetVelocity()).withTimeout(1.0),
        // Once ready, feed the shooter while maintaining flywheel speed
        feedFly(feeder).alongWith(smartFlyWheel(flyWheel)));
  }
}
