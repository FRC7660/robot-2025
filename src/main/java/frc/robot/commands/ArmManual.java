// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class ArmManual extends Command {
  private final Elevator m_elevator;
  private final Arm m_arm;

  private final Constants.Arm.Direction direction;
  private double limitOut, limitIn;
  private boolean movementAllowed = false;

  public ArmManual(Arm arm, Elevator elevator, Constants.Arm.Direction direction) {
    m_arm = arm;
    m_elevator = elevator;
    this.direction = direction;
    addRequirements(elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.hold();

    limitOut = Constants.Arm.forewardLimit;
    if (!m_elevator.isAtBottom()) {
      limitIn = Constants.Arm.safePosIn;
    } else {
      limitIn = Constants.Arm.zeroPos;
    }

    String dirStr = "out";
    if (direction == Constants.Arm.Direction.OUT) {
      m_arm.manualOut();
    } else {
      dirStr = "in";
      m_arm.manualIn();
    }

    System.out.println("Arm Manual-" + dirStr + " limits(" + limitIn + ", " + limitOut + ")");
  }

  @Override
  public void execute() {
    m_arm.setTargetWhileManual(m_arm.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.holdCurrentPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (direction == Constants.Arm.Direction.OUT) {
      return m_arm.getPosition() <= limitOut;
    }
    return m_arm.getPosition() >= limitIn;
  }
}
