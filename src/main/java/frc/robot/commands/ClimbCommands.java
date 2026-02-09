// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.structures.Climber;

/** Add your docs here. */
public class ClimbCommands {
    public ClimbCommands() {}

    public static Command simpleClimbCommand(Climber climber) {
        return Commands.run(() -> climber.setClimberVolts(6), climber)
            .handleInterrupt(() -> climber.setClimberVolts(0));
    }
    public static Command simpleDeclimbCommand(Climber climber) {
        return Commands.run(() -> climber.setClimberVolts(-6), climber)
            .handleInterrupt(() -> climber.setClimberVolts(0));
    }

    public static Command manualClimbCommand(Climber climber, DoubleSupplier set) {
        double y = set.getAsDouble();
        return Commands.run(() -> climber.manualClimb(y), climber)
            .handleInterrupt(() -> climber.manualClimb(0));
    }
}
