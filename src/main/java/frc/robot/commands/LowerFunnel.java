// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Funnel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LowerFunnel extends Command {
  /** Creates a new LiftIndex. */
  private final Funnel funnel;

  private final Climb climb;

  public LowerFunnel(Funnel funnel, Climb climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.funnel = funnel;
    this.climb = climb;
    addRequirements(funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Funnel is lowering");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climb.isOut()) {
      funnel.wind();
    } else {
      funnel.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnel.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return funnel.limitHit();
  }
}
