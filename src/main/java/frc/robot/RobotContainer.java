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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorState;
import frc.robot.commands.ArmPIDTest;
import frc.robot.commands.ClimbPrepRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.LowerClimb;
import frc.robot.commands.LowerFunnel;
import frc.robot.commands.RaiseClimb;
import frc.robot.commands.SwitchVideo;
import frc.robot.commands.TestAuto;
import frc.robot.commands.releaseCoral;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.LEDsubsystem.LEDlive;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
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
  private final LEDlive ledLive;
  private final Elevator elevator;
  private SwerveDriveSimulation driveSimulation = null;
  private final Funnel funnel = new Funnel();
  private final Arm arm = new Arm();
  private final Climb climb = new Climb();
  private final Claw claw = new Claw();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandGenericHID buttonBox = new CommandGenericHID(1);
  private final CommandXboxController testController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    DriverStation.silenceJoystickConnectionWarning(Constants.currentMode == Constants.Mode.SIM);

    ledLive = new LEDlive();
    elevator = new Elevator();
    drive = new Drive();

    // Set up auto routines
    // new EventTrigger("BytingEventMarker").onTrue(testEventMarker);
    TestAuto testCommand = new TestAuto("Byting Command");
    TestAuto testEventMarker = new TestAuto("Byting Event Marker");
    // NamedCommands.registerCommand("Test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("BytingCommand", testCommand);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Default command for Elevator
    // elevator.setDefaultCommand(
    //     elevator.runManualCommand(() -> MathUtil.applyDeadband(testController.getLeftY(), 0.1)));

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
    configurebuttonBox();
    arm.setDefaultCommand(arm.manualArm(testController::getLeftY));
    // Default command, normal field-relative drive

    // driverController.a().onTrue(new LiftFunnel(funnel));
    driverController.a().whileTrue(new LowerClimb(climb));
    driverController.b().whileTrue(new RaiseClimb(climb));
    driverController.x().onTrue(new LowerFunnel(funnel, climb));
    // driverController.y().whileTrue(new LiftFunnel(funnel, climb));
    driverController.povRight().whileTrue(new ClimbPrepRoutine(climb, funnel));

    driverController
        .leftTrigger(0.1)
        .whileTrue(
            DriveCommands.strafe(drive, true, () -> driverController.getLeftTriggerAxis() * 0.5));
    driverController
        .rightTrigger(0.1)
        .whileTrue(
            DriveCommands.strafe(drive, false, () -> driverController.getRightTriggerAxis() * 0.5));

    // driverController.rightBumper().whileTrue(DriveCommands.strafe(drive,false,() -> 0.1));

    // switch camera video displayed on Elastic
    driverController.y().onTrue(new SwitchVideo());

    testController.a().whileTrue(new ArmPIDTest(arm));

    driverController.povUp().onTrue(new IntakeCoral(claw));
    driverController.povDown().onTrue(new releaseCoral(claw));

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
                    new Pose2d(
                        drive.getPose().getTranslation(), new Rotation2d(Math.PI))); // zero gyro
    driverController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  private void setUpBoxButton(int inputButton) {
    Trigger buttonXtrigger = buttonBox.button(inputButton);
    String buttonName;
    Constants.ElevatorState height;
    boolean left;
    switch (inputButton) {

      // LEFT SIDE PRESETS
      case Constants.ButtonBox.bottomLeft:
        buttonName = "bottom left";
        height = ElevatorState.L1;
        left = true;
        break;
      case Constants.ButtonBox.lowerLeft:
        buttonName = "lower left";
        height = ElevatorState.L2;
        left = true;
        break;
      case Constants.ButtonBox.upperLeft:
        buttonName = "upper left";
        height = ElevatorState.L3;
        left = true;
        break;
      case Constants.ButtonBox.topLeft:
        buttonName = "top left";
        height = ElevatorState.L4;
        left = true;
        break;

      // RIGHT SIDE PRESETS
      case Constants.ButtonBox.bottomRight:
        buttonName = "bottom right";
        height = ElevatorState.L1;
        left = false;
        break;
      case Constants.ButtonBox.lowerRight:
        buttonName = "lower right";
        height = ElevatorState.L2;
        left = false;
        break;
      case Constants.ButtonBox.upperRight:
        buttonName = "upper right";
        height = ElevatorState.L3;
        left = false;
        break;
      case Constants.ButtonBox.topRight:
        buttonName = "top right";
        height = ElevatorState.L4;
        left = false;
        break;

      // ERROR
      default:
        buttonName = "NONEXISTENT";
        height = ElevatorState.ZERO;
        left = false;
        break;
    }
    buttonXtrigger.whileTrue(Commands.run(() -> elevator.setState(height), elevator));
    buttonXtrigger.onTrue(new PrintCommand(buttonName + " pressed (BBOX)"));
    buttonXtrigger.onFalse(new PrintCommand(buttonName + " released (BBOX)"));
  }

  private void configurebuttonBox() {
    setUpBoxButton(Constants.ButtonBox.bottomLeft);
    setUpBoxButton(Constants.ButtonBox.lowerLeft);
    setUpBoxButton(Constants.ButtonBox.upperLeft);
    setUpBoxButton(Constants.ButtonBox.topLeft);
    setUpBoxButton(Constants.ButtonBox.bottomRight);
    setUpBoxButton(Constants.ButtonBox.lowerRight);
    setUpBoxButton(Constants.ButtonBox.upperRight);
    setUpBoxButton(Constants.ButtonBox.topRight);

    Trigger tp1 = buttonBox.button(Constants.ButtonBox.p1);
    tp1.onTrue(new PrintCommand("p1 Pressed"));
    tp1.onFalse(new PrintCommand("p1 Released"));
    Trigger tp2 = buttonBox.button(Constants.ButtonBox.p2);
    tp2.onTrue(new PrintCommand("p2 Pressed"));
    tp2.onFalse(new PrintCommand("p2 Released"));

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
