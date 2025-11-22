// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeGoToPos extends Command {
  private final Intake intake;
  private final double targetPosition;

  /**
   * Creates a new IntakeGoToPos command.
   *
   * @param intake The intake subsystem
   * @param position Target position in rotations
   */
  public IntakeGoToPos(Intake intake, double position) {
    this.intake = intake;
    this.targetPosition = position;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    // Validate target is within limits
    if (targetPosition < Constants.Intake.lowerLimit
        || targetPosition > Constants.Intake.upperLimit) {
      System.out.println(
          "IntakeGoToPos: Target position "
              + targetPosition
              + " is outside limits ["
              + Constants.Intake.lowerLimit
              + ", "
              + Constants.Intake.upperLimit
              + "]");
      cancel();
      return;
    }

    // Set the target position
    intake.setTarget(targetPosition);
  }

  @Override
  public void execute() {
    // PID control happens in subsystem periodic()
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      intake.hold();
    }
  }

  @Override
  public boolean isFinished() {
    return intake.isAtGoal();
  }
}
