// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToPos extends Command {
  Elevator m_elevator;
  Arm m_arm;
  Constants.ElevatorState m_position;

  /** Creates a new ElevatorGoToPos. */
  public ElevatorGoToPos(Elevator elevator, Arm arm, Constants.ElevatorState position) {
    m_elevator = elevator;
    m_arm = arm;
    m_position = position;
    addRequirements(m_elevator, m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.holdCurrentPosition();

    System.out.println("Elevator Preset attempt: " + m_position);
    if (!m_arm.isInSafeZone()) {
      System.out.println("ELEVATOR PRESET CANCELED");
      this.cancel();
    }

    m_elevator.setState(m_position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.isAtGoal();
  }
}
