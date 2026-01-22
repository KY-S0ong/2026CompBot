// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
// Modified by FRC 3958
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.structures.Feeder;
import frc.robot.subsystems.structures.Flywheel;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private VisionSystem visionSystem = new VisionSystem();
  private Flywheel shootIntake = new Flywheel();
  private Feeder feeder = new Feeder();

  // Controller
  public static final Joystick LdriveJoystick = new Joystick(0);
  public static final Joystick RdriveJoystick = new Joystick(1);
  private static final CommandXboxController opperatorController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoNamedCommands();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveBindings();
    shootBindings();
  }

  private void driveBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -LdriveJoystick.getY(),
            () -> -LdriveJoystick.getX(),
            () -> -RdriveJoystick.getX(),
            () -> true));

    new JoystickButton(LdriveJoystick, 3)
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -LdriveJoystick.getY(),
                () -> -LdriveJoystick.getX(),
                () -> -RdriveJoystick.getX(),
                () -> false));

    // Hub-relative drive when RB is held
    new JoystickButton(LdriveJoystick, 2)
        .whileTrue(
            DriveCommands.joystickHubDrive(
                drive,
                () -> -LdriveJoystick.getY(),
                () -> -LdriveJoystick.getX(),
                () -> drive.getPose()));

    // Lock to 0° when A button is held
    new JoystickButton(RdriveJoystick, 2)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -LdriveJoystick.getY(),
                () -> -LdriveJoystick.getX(),
                () -> Rotation2d.kZero));
    // Lock to 180° when A button is held
    new JoystickButton(RdriveJoystick, 4)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -LdriveJoystick.getY(),
                () -> -LdriveJoystick.getX(),
                () -> Rotation2d.k180deg));
    // Switch to X pattern when X button is pressed
    new JoystickButton(RdriveJoystick, 5).onTrue(Commands.runOnce(drive::stopWithX, drive));
    // Reset gyro to 0° when B button is pressed
    new JoystickButton(RdriveJoystick, 3)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  private void shootBindings() {
    new JoystickButton(LdriveJoystick, 1).whileTrue(ShootCommands.rampFlyWheel(shootIntake, .5));
    // new JoystickButton(RdriveJoystick, 1).whileTrue(ShootCommands.intake(shootIntake));
  }

  private void autoNamedCommands() {
    NamedCommands.registerCommand(
        "launchSequence", ShootCommands.autolaunchSequence(shootIntake, feeder));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
