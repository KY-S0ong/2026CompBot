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
import frc.robot.subsystems.structures.Intake;

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

  public static Command runRollers(Intake intake, double volts) {
    return Commands.run(() -> intake.runIntake(volts), intake)
        .handleInterrupt(() -> intake.stopIntake());
  }

  public static SequentialCommandGroup smartRunRollers(Intake intake, double volts) {
    return new SequentialCommandGroup(
        runRollers(intake, -4.5).withTimeout(.2), runRollers(intake, volts));
  }

  public static Command feedFly(Feeder feeder) {
    return Commands.run(() -> feeder.feedShooter(-6), feeder)
        .handleInterrupt(() -> feeder.stopFeeder());
  }

  public static Command smartFeedFly(Feeder feeder, Flywheel flywheel) {
    return Commands.run(() -> feeder.smartFeed(-6, flywheel), feeder)
        .handleInterrupt(() -> feeder.stopFeeder());
  }

  public static ParallelRaceGroup intake(Flywheel flyWheel, Intake intake, Feeder feeder) {
    return new ParallelRaceGroup(
        runRollers(intake, 6),
        Commands.run(() -> feeder.feedShooter(9), feeder)
            .handleInterrupt(() -> feeder.stopFeeder()));
  }

  public static SequentialCommandGroup smartIntake(
      Flywheel flywheel, Intake intake, Feeder feeder) {
    return new SequentialCommandGroup(
        extake(intake, feeder).withTimeout(0.05), intake(flywheel, intake, feeder));
  }

  public static ParallelRaceGroup extake(Intake intake, Feeder feeder) {
    return new ParallelRaceGroup(
        runRollers(intake, -8).handleInterrupt(() -> intake.stopIntake()),
        Commands.run(() -> feeder.feedShooter(-9), feeder)
            .handleInterrupt(() -> feeder.stopFeeder()));
  }

  public static ParallelRaceGroup unJam(Flywheel flyWheel, Feeder feeder, Intake intake) {
    return new ParallelRaceGroup(
        rampFlyWheel(flyWheel, 12).handleInterrupt(() -> flyWheel.stopFlyWheel()),
        Commands.run(() -> feeder.feedShooter(12), feeder)
            .handleInterrupt(() -> feeder.stopFeeder()),
        runRollers(intake, 12));
  }

  /* Eventually implement distance calibrations */

  public static SequentialCommandGroup launchSequence(
      Flywheel flyWheel, Feeder feeder, Intake intake) {
    return new SequentialCommandGroup(
        // rampFlyWheel(flyWheel, 5).withTimeout(1.0),
        smartFlyWheel(flyWheel).alongWith(smartRunRollers(intake, 10)).withTimeout(2.0),
        feedFly(feeder)
            .alongWith(rampFlyWheel(flyWheel, 5) /*smartFlyWheel(flyWheel)*/)
            .alongWith(runRollers(intake, 10)));
  }

  public static SequentialCommandGroup TlaunchSequence(
      Flywheel flyWheel, Feeder feeder, Intake intake) {
    return new SequentialCommandGroup(
        smartFlyWheel(flyWheel).alongWith(smartRunRollers(intake, 10)).withTimeout(0.4),
        smartFeedFly(feeder, flyWheel)
            .alongWith(smartFlyWheel(flyWheel))
            .alongWith(runRollers(intake, 10)));
  }

  public static SequentialCommandGroup overrideLaunch(
      Flywheel flyWheel, Feeder feeder, Intake intake, double velocity) {
    return new SequentialCommandGroup(
        Commands.run(() -> flyWheel.setFlyVelocity(velocity), flyWheel)
            .alongWith(smartRunRollers(intake, 10))
            .withTimeout(2.0),
        feedFly(feeder)
            .alongWith(Commands.run(() -> flyWheel.setFlyVelocity(velocity), flyWheel))
            .alongWith(runRollers(intake, velocity)));
  }

  /*public static SequentialCommandGroup voltLaunch(Flywheel flyWheel, Feeder feeder, double volts) {
    return new SequentialCommandGroup(
        rampFlyWheel(flyWheel, volts).withTimeout(.35),
        feedFly(feeder).alongWith(rampFlyWheel(flyWheel, volts)));
  }*/

  public static Command testPID(Flywheel flywheel) {
    return smartFlyWheel(flywheel);
  }

  public static SequentialCommandGroup autolaunchSequence(
      Flywheel flyWheel, Feeder feeder, Intake intake) {
    return new SequentialCommandGroup(
        smartFlyWheel(flyWheel).alongWith(smartRunRollers(intake, 10)).withTimeout(1.25),
        feedFly(feeder).alongWith(smartFlyWheel(flyWheel)).alongWith(runRollers(intake, 10)));
  }
}
