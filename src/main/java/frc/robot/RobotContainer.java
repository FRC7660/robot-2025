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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorState;
import frc.robot.commands.ArmGoToPos;
import frc.robot.commands.ArmManual;
import frc.robot.commands.ClimbPrepRoutine;
import frc.robot.commands.DriveForTime;
import frc.robot.commands.ElevatorGoToPos;
import frc.robot.commands.ElevatorManual;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.LowerClimb;
import frc.robot.commands.LowerFunnel;
import frc.robot.commands.RaiseClimb;
import frc.robot.commands.RemoveAlgae;
import frc.robot.commands.SwitchVideo;
import frc.robot.commands.TestAuto;
import frc.robot.commands.releaseCoral;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.LEDsubsystem.LEDlive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final LEDlive ledLive;
  private final Elevator elevator;
  private final Funnel funnel = new Funnel();
  private final Arm arm = new Arm();
  private final Climb climb = new Climb();
  private final Claw claw = new Claw();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandGenericHID buttonBox = new CommandGenericHID(1);
  private final CommandXboxController testController = new CommandXboxController(2);

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "7660"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverController.getLeftY() * 1,
              () -> driverController.getLeftX() * 1)
          .withControllerRotationAxis(driverController::getRightX)
          .deadband(Constants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  /*The driver input inversion seems unnecessary if everything else is right, so maybe fix later */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(
              () ->
                  DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                      ? driverController.getRightX()
                      : -driverController.getRightX(),
              () ->
                  DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                      ? driverController.getRightY()
                      : -driverController.getRightY())
          .headingWhile(true);

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverController.getLeftY(),
              () -> -driverController.getLeftX())
          .withControllerRotationAxis(() -> driverController.getRawAxis(2))
          .deadband(Constants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(0));

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    DriverStation.silenceJoystickConnectionWarning(Constants.currentMode == Constants.Mode.SIM);
    NamedCommands.registerCommand("Test", Commands.print("I EXIST"));

    ledLive = new LEDlive();
    elevator = new Elevator();

    // Set up auto routines
    // new EventTrigger("BytingEventMarker").onTrue(testEventMarker);
    TestAuto testCommand = new TestAuto("Byting Command");
    TestAuto testEventMarker = new TestAuto("Byting Event Marker");
    // NamedCommands.registerCommand("Test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("BytingCommand", testCommand);
    NamedCommands.registerCommand("Arm Out", armToScorePos());
    NamedCommands.registerCommand("Elevator L2", elevatorL2());
    NamedCommands.registerCommand("Arm Shoot Pos", armToScorePos());
    NamedCommands.registerCommand("Shoot Coral", new releaseCoral(claw));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("Drive Back", new DriveForTime(drivebase, -1, 0, 1));
    autoChooser.addOption("Relative drive to L4", autoDriveForL4());

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

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard =
        drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard =
        drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard =
        drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    if (Robot.isSimulation()) {
      configureSimBindings();
    }

    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(
          driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverController
          .leftBumper()
          .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverController.back().whileTrue(drivebase.centerModulesCommand());
    }

    // start: hamburger/menu/right tiny button
    // back: two squares/view/left tiny button
    driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    driverController
        .back()
        .onTrue(
            new SequentialCommandGroup(
                new ClimbPrepRoutine(climb, funnel),
                new ArmGoToPos(arm, elevator, Constants.Arm.climbPos)));
    // driverController.back().onTrue(new ClimbPrepRoutine(climb, funnel));

    driverController.a().onTrue(new IntakeCoral(claw));
    driverController.b().onTrue(new releaseCoral(claw));
    driverController.x().onTrue(goToHome());
    driverController.y().onTrue(new SwitchVideo());

    driverController.rightBumper().whileTrue(new RemoveAlgae(claw));
    driverController
        .leftBumper()
        .whileTrue(
            driveRobotRelative(() -> Constants.strafeSpeedMultiplier * 0.1, () -> 0, () -> 0));

    driverController
        .povUp()
        .whileTrue(new ElevatorManual(elevator, arm, Constants.Elevator.Direction.UP));
    driverController
        .povDown()
        .whileTrue(new ElevatorManual(elevator, arm, Constants.Elevator.Direction.DOWN));
    driverController
        .povRight()
        .whileTrue(new ArmManual(arm, elevator, Constants.Arm.Direction.OUT));
    driverController.povLeft().whileTrue(new ArmManual(arm, elevator, Constants.Arm.Direction.IN));

    driverController
        .leftTrigger(0.04)
        .whileTrue(
            driveRobotRelative(
                () -> 0,
                () -> -Constants.strafeSpeedMultiplier * driverController.getLeftTriggerAxis(),
                () -> 0));
    driverController
        .rightTrigger(0.04)
        .whileTrue(
            driveRobotRelative(
                () -> 0,
                () -> Constants.strafeSpeedMultiplier * driverController.getRightTriggerAxis(),
                () -> 0));

    testController.a().whileTrue(armToScorePos());
    testController.x().whileTrue(new ArmGoToPos(arm, elevator, Constants.Arm.zeroPos));
    testController.y().whileTrue(new ElevatorGoToPos(elevator, arm, ElevatorState.L4));
    testController.b().whileTrue(new ElevatorGoToPos(elevator, arm, ElevatorState.ZERO));

    // // Reset gyro / odometry
    // final Runnable resetGyro =
    //     Constants.currentMode == Constants.Mode.SIM // this is an IF statement
    //         // simulation
    //         ? () ->
    //             drive.resetOdometry(
    //                 driveSimulation
    //                     .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose
    // during
    //         // real
    //         : () ->
    //             drive.resetOdometry(
    //                 new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    // driverController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  private void configureSimBindings() {
    Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
    // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
    driveDirectAngleKeyboard.driveToPose(
        () -> target,
        new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
        new ProfiledPIDController(
            5, 0, 0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
    driverController
        .start()
        .onTrue(
            Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    driverController
        .button(2)
        .whileTrue(
            Commands.runEnd(
                () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

    //      driverXbox.b().whileTrue(
    //          drivebase.driveToPose(
    //              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                              );
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
        height = ElevatorState.ZERO;
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

      // RIGHT SIDE PRESETS - Not Used Right Now
      case Constants.ButtonBox.bottomRight:
        buttonName = "bottom right";
        height = ElevatorState.ZERO;
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
    if (height == Constants.ElevatorState.L4){
      buttonXtrigger.onTrue(
          new SequentialCommandGroup(
              new ArmGoToPos(arm, elevator, Constants.Arm.scorePos),
              new ElevatorGoToPos(elevator, arm, height),
              new ArmGoToPos(
                  arm,
                  elevator,
                  Constants.Arm.scorePosL4),
              new DriveForTime(drivebase,0.3,0,0.3)));
    } else {
      buttonXtrigger.onTrue(
          new SequentialCommandGroup(
              new ArmGoToPos(arm, elevator, Constants.Arm.scorePos),
              new ElevatorGoToPos(elevator, arm, height),
              new ArmGoToPos(
                  arm,
                  elevator,
                  Constants.Arm.scorePos)));
    }
    buttonXtrigger.onTrue(new PrintCommand(buttonName + " pressed (BBOX)"));
    buttonXtrigger.onFalse(new PrintCommand(buttonName + " released (BBOX)"));
  }

  private void configurebuttonBox() {
    // setUpBoxButton(Constants.ButtonBox.bottomLeft);
    Trigger buttonBLtrigger = buttonBox.button(Constants.ButtonBox.bottomLeft);
    buttonBLtrigger.onTrue(goToHome());

    setUpBoxButton(Constants.ButtonBox.lowerLeft);
    setUpBoxButton(Constants.ButtonBox.upperLeft);
    setUpBoxButton(Constants.ButtonBox.topLeft);

    buttonBox.button(Constants.ButtonBox.bottomRight).whileTrue(new LowerClimb(climb));
    buttonBox.button(Constants.ButtonBox.lowerRight).whileTrue(new RaiseClimb(climb));
    buttonBox.button(Constants.ButtonBox.upperRight).whileTrue(new LowerFunnel(funnel, climb));

    // Button Board's Dpad, axis 0: up/down, axis 1: right/left
    buttonBox
        .axisGreaterThan(0, 0.5)
        .whileTrue(new ElevatorManual(elevator, arm, Constants.Elevator.Direction.UP));
    buttonBox
        .axisLessThan(0, -0.5)
        .whileTrue(new ElevatorManual(elevator, arm, Constants.Elevator.Direction.DOWN));
    buttonBox
        .axisGreaterThan(1, 0.5)
        .whileTrue(new ArmManual(arm, elevator, Constants.Arm.Direction.OUT));
    buttonBox
        .axisLessThan(1, -0.5)
        .whileTrue(new ArmManual(arm, elevator, Constants.Arm.Direction.IN));

    buttonBox
        .button(Constants.ButtonBox.p1)
        .onTrue(new DriveForTime(drivebase,0.3,0,0.3));

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

    drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public SwerveSubsystem getDrive() {
    return drivebase;
  }

  public void setMotorBrakeMode(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  private Command armToScorePos() {
    return new ArmGoToPos(arm, elevator, Constants.Arm.scorePos);
  }

  private Command elevatorL2() {
    return new ElevatorGoToPos(elevator, arm, ElevatorState.L2);
  }

  private Command goToHome() {
    return new SequentialCommandGroup(
        armToScorePos(),
        new ElevatorGoToPos(elevator, arm, ElevatorState.ZERO),
        new ArmGoToPos(arm, elevator, Constants.Arm.zeroPos));
  }

  /**
   * Drives the with the given velocities robot oriented. Sets speeds to 0 on end.
   *
   * @param vx The x component of the velocity in meters/second. Positive is forward.
   * @param vy The y component of the velocity in meters/second. Positive is to the left.
   * @param omega The rotational component of the velocity in radians/second. Positive is
   *     counter-clockwise.
   * @return The command to drive at the provided speeds.
   */
  private Command driveRobotRelative(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return Commands.runEnd(
        () ->
            drivebase.drive(
                new ChassisSpeeds(vx.getAsDouble(), vy.getAsDouble(), omega.getAsDouble())),
        () -> drivebase.drive(new ChassisSpeeds(0, 0, 0)),
        drivebase);
  }

  private Command autoDriveForL4() {
    return new SequentialCommandGroup(
        new DriveForTime(drivebase, -1, 0, 1),
        drivebase.driveToDistanceCommand(0.1, 0.1),
        // new DriveForTime(drivebase, 0.2, 0, 0),
        new ArmGoToPos(arm, elevator, Constants.Arm.scorePos),
        new ElevatorGoToPos(elevator, arm, Constants.ElevatorState.L4),
        new ArmGoToPos(arm, elevator, Constants.Arm.scorePosL4),
        new releaseCoral(claw));
  }
}
