// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmGoToPos extends Command {
  private final Arm arm;
  private final double targetPos;

  /** Creates a new ArmGoToPos. */
  public ArmGoToPos(Arm arm, double Pos) { // ADD ELEVATOR TO REQUIREMENTS
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    targetPos = Pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setTarget(targetPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    return;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atTargetPos();
  }
}
