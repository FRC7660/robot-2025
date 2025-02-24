// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoscore;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestStrafe extends Command {
  private final Drive drive;
  boolean isLeft;
  Double magnitude;

  /** Creates a new TestStrafe. */
  public TestStrafe(Drive driveIntake, boolean left, Double mag) {
    // Use addRequirements() here to declare subsystem dependencies.

    drive = driveIntake;
    isLeft = left;
    magnitude = mag;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("TEST STRAFE REACHED");
    DriveCommands.strafe(drive, isLeft, () -> magnitude);
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
