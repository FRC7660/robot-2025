// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveForTime extends Command {
  private final SwerveSubsystem drive;
  private final Timer timer = new Timer();
  private final double speedx;
  private final double speedy;
  private final double time;

  /** Creates a new DriveForTime. */
  public DriveForTime(SwerveSubsystem drive, double speedx, double speedy, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.speedx = speedx;
    this.speedy = speedy;
    this.time = time;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.drive(new ChassisSpeeds(speedx, speedy, 0));
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(new ChassisSpeeds(speedx, speedy, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= time;
  }
}
