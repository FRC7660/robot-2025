// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.Direction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorManual extends Command {
  private final Elevator m_elevator;
  private final Arm m_arm;
  private Direction direction;
  private double limitUp;
  private double limitDown;

  public ElevatorManual(Elevator elevator, Arm arm, Constants.Elevator.Direction direction) {
    m_arm = arm;
    m_elevator = elevator;
    this.direction = direction;
    addRequirements(m_elevator, m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.holdCurrentPosition();

    limitUp = Constants.Elevator.upperLimit;
    if (m_arm.isInSafeZone()) {
      limitDown = Constants.Elevator.lowerLimit;
    } else {
      System.out.println("ELEVATOR MANUAL CANCELED");
      m_elevator.hold();
      this.cancel();
      return;
    }

    String dirStr = "up";
    if (direction == Constants.Elevator.Direction.UP) {
      m_elevator.manualUp();
    } else {
      dirStr = "down";
      m_elevator.manualDown();
    }

    System.out.println("Elevator Manual-" + dirStr + " limits(" + limitUp + ", " + limitDown + ")");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (direction == Constants.Elevator.Direction.UP) {
      return m_elevator.getHeight() >= limitUp;
    }
    return m_elevator.getPosition() <= limitDown;
  }
}
