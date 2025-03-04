// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Strafe extends Command {
  private final SwerveSubsystem swerve;
  private boolean isLeft;
  private DoubleSupplier strafeSpeed;
  private ProfiledPIDController controller;

  /** Creates a new Strafe. */
  public Strafe(SwerveSubsystem swerve, DoubleSupplier strafeSpeed, boolean isLeft) {
    this.swerve = swerve;
    this.strafeSpeed = strafeSpeed;
    this.isLeft = isLeft;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = new ProfiledPIDController(5, 0, 0.4, new Constraints(16, 20));
    controller.reset(swerve.getHeading().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d currentRotation = swerve.getHeading();
    Translation2d linearVelocity =
        swerve.getStrafeVelocity(currentRotation, strafeSpeed.getAsDouble());
    double omega =
        controller.calculate(swerve.getHeading().getRadians(), currentRotation.getRadians());

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * swerve.getMaxLinearSpeed(),
            linearVelocity.getY() * swerve.getMaxLinearSpeed(),
            omega);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? swerve.getHeading().plus(new Rotation2d(isLeft ? -Math.PI / 2 : Math.PI / 2))
                : swerve.getHeading());

    swerve.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
