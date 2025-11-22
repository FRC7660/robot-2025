// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Intake.Direction;
import frc.robot.subsystems.Intake;

public class IntakeManual extends Command {
  private final Intake intake;
  private final Direction direction;
  private double limitForward;
  private double limitReverse;

  /**
   * Creates a new IntakeManual command.
   *
   * @param intake The intake subsystem
   * @param direction Direction to move (FORWARD or REVERSE)
   */
  public IntakeManual(Intake intake, Direction direction) {
    this.intake = intake;
    this.direction = direction;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    limitForward = Constants.Intake.upperLimit;
    limitReverse = Constants.Intake.lowerLimit;

    String dirStr = "forward";
    if (direction == Direction.FORWARD) {
      intake.manualForward();
    } else {
      dirStr = "reverse";
      intake.manualReverse();
    }

    System.out.println(
        "Intake Manual-"
            + dirStr
            + " limits("
            + limitForward
            + ", "
            + limitReverse
            + ")");
  }

  @Override
  public void execute() {
    // Manual control happens in subsystem periodic()
  }

  @Override
  public void end(boolean interrupted) {
    intake.hold();
  }

  @Override
  public boolean isFinished() {
    if (direction == Direction.FORWARD) {
      return intake.getPosition() >= limitForward;
    }
    return intake.getPosition() <= limitReverse;
  }
}
