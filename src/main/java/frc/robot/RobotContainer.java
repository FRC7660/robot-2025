// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.LiftFunnel;
import frc.robot.commands.LowerClimb;
import frc.robot.commands.TestAuto;
import frc.robot.commands.releaseCoral;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.LEDsubsystem.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
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
  private final Vision vision;
  private final LEDlive ledLive;
  private final Elevator elevator;
  private SwerveDriveSimulation driveSimulation = null;
  private final Funnel funnel = new Funnel();
  private final Climb climb = new Climb();
  private final Claw claw = new Claw();

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController coDriverController = new CommandXboxController(1);

  private final XboxController driver = new XboxController(0);
  private final XboxController coDriver = new XboxController(1);
  private final CommandGenericHID buttonBox = new CommandGenericHID(2);

  // Dashboard inputs

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    ledLive = new LEDlive();
    elevator = new Elevator();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                (pose) -> {});

        this.vision =
            new Vision(
                drive,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

        break;
      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        break;
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

        // Configure the button bindings
        configureButtonBindings();

        // Configure the elevator
        configureElevatorHeights();
        break;
    }

    // Set up auto routines
    // new EventTrigger("BytingEventMarker").onTrue(testEventMarker);
    TestAuto testCommand = new TestAuto("Byting Command");
    TestAuto testEventMarker = new TestAuto("Byting Event Marker");
    // NamedCommands.registerCommand("Test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("BytingCommand", testCommand);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Default command for Elevator
    elevator.setDefaultCommand(
        elevator.runManualCommand(
            () -> MathUtil.applyDeadband(coDriverController.getLeftY(), 0.1)));

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

    // Configure the elevator
    configureElevatorHeights();
  }

  private void configureElevatorHeights() {
    // Set height to ZERO when coDriver button 1 is pressed
    coDriverController
        .a()
        .whileTrue(Commands.run(() -> elevator.setState(Constants.ElevatorState.L1), elevator));
    coDriverController
        .x()
        .whileTrue(Commands.run(() -> elevator.setState(Constants.ElevatorState.L2), elevator));
    coDriverController
        .y()
        .whileTrue(Commands.run(() -> elevator.setState(Constants.ElevatorState.L3), elevator));
    coDriverController
        .b()
        .whileTrue(Commands.run(() -> elevator.setState(Constants.ElevatorState.L4), elevator));
    coDriverController
        .button(9)
        .whileTrue(Commands.run(() -> elevator.setState(Constants.ElevatorState.ZERO), elevator));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configurebuttonBox();
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        Constants.absoluteDrive
            ? DriveCommands.joystickDriveAtAngle(
                // Absolute Drive
                drive,
                () -> controller.getLeftX(),
                () -> -controller.getLeftY(),
                () -> -controller.getRightX(),
                () -> controller.getRightY())
            :
            // Field Drive
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));

    JoystickButton a = new JoystickButton(driver, XboxController.Button.kA.value);
    a.onTrue(new LiftFunnel(funnel));

    JoystickButton b = new JoystickButton(driver, XboxController.Button.kB.value);
    b.onTrue(new LowerClimb(climb));

    JoystickButton x = new JoystickButton(driver, XboxController.Button.kX.value);
    x.onTrue(new releaseCoral(claw));

    JoystickButton y = new JoystickButton(driver, XboxController.Button.kY.value);
    y.onTrue(new IntakeCoral(claw));

    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM // this is an IF statement
            // simulation
            ? () ->
                drive.resetOdometry(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // real
            : () ->
                drive.resetOdometry(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  private void configurebuttonBox() {
    Trigger button1 = buttonBox.button(Constants.ButtonBox.bottomLeft);
    button1.onTrue(new PrintCommand("Bottom Left pressed"));
    button1.onFalse(new PrintCommand("Bottom Left released"));
    Trigger button2 = buttonBox.button(Constants.ButtonBox.lowerLeft);
    button2.onTrue(new PrintCommand("Lower Left Pressed"));
    button2.onFalse(new PrintCommand("Lower Left Released"));
    Trigger button3 = buttonBox.button(Constants.ButtonBox.upperLeft);
    button3.onTrue(new PrintCommand("Upper Left Pressed"));
    button3.onFalse(new PrintCommand("Upper Left Released"));
    Trigger button4 = buttonBox.button(Constants.ButtonBox.topLeft);
    button4.onTrue(new PrintCommand("Top Left Pressed"));
    button4.onFalse(new PrintCommand("Top Left Released"));
    Trigger button5 = buttonBox.button(Constants.ButtonBox.bottomRight);
    button5.onTrue(new PrintCommand("Bottom Right Pressed"));
    button5.onFalse(new PrintCommand("Bottom Right Released"));
    Trigger button6 = buttonBox.button(Constants.ButtonBox.lowerRight);
    button6.onTrue(new PrintCommand("Lower Right Pressed"));
    button6.onFalse(new PrintCommand("Lower Right Released"));
    Trigger button7 = buttonBox.button(Constants.ButtonBox.upperRight);
    button7.onTrue(new PrintCommand("Upper Right Pressed"));
    button7.onFalse(new PrintCommand("Upper Right Released"));
    Trigger button8 = buttonBox.button(Constants.ButtonBox.topRight);
    button8.onTrue(new PrintCommand("Top Right Pressed"));
    button8.onFalse(new PrintCommand("Top Right Released"));
    Trigger button9 = buttonBox.button(Constants.ButtonBox.p1);
    button9.onTrue(new PrintCommand("p1 Pressed"));
    button9.onFalse(new PrintCommand("p1 Released"));
    Trigger button10 = buttonBox.button(Constants.ButtonBox.p2);
    button10.onTrue(new PrintCommand("p2 Pressed"));
    button10.onFalse(new PrintCommand("p2 Released"));

    /*
    ArrayList<Trigger> buttons = new ArrayList<Trigger>(10);
    for (int i = 0; i < buttons.size(); i++) {
        Trigger button = buttonBox.button(i+1);
        button.onTrue(new PrintCommand("Button " + i +1 + " Pressed"));
        button.onFalse(new PrintCommand("Button " + i +1 +"" Released"));

    }
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  public Drive getDrive() {
    return drive;
  }
}
