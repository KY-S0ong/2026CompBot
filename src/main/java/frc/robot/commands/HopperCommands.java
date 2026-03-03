package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.structures.Hopper;
import java.util.function.DoubleSupplier;

public class HopperCommands {

  private HopperCommands() {}

  public static Command open(Hopper hopper) {
    return Commands.run(() -> hopper.openHopper(), hopper).handleInterrupt(() -> hopper.stop());
  }

  public static Command close(Hopper hopper) {
    return Commands.run(() -> hopper.closeHopper(), hopper).handleInterrupt(() -> hopper.stop());
  }

  public static Command moveToPosition(Hopper hopper, double position) {
    return Commands.run(() -> hopper.moveToPosition(position), hopper)
        .handleInterrupt(() -> hopper.stop());
  }

  public static Command manual(Hopper hopper, DoubleSupplier speedSupplier) {
    return Commands.run(() -> hopper.manualControl(speedSupplier.getAsDouble()), hopper)
        .handleInterrupt(() -> hopper.stop());
  }

  public static Command stop(Hopper hopper) {
    return Commands.runOnce(() -> hopper.stop(), hopper);
  }
}
