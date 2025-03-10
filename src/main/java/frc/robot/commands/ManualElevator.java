// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ManualElevator extends Command {
  private final Elevator m_elevator;
  private DoubleSupplier m_speed;
  private final Arm m_arm;

  public ManualElevator(Elevator elevator, DoubleSupplier speed, Arm arm) {
    m_arm = arm;
    m_elevator = elevator;
    m_speed = speed;
    addRequirements(elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_arm.setPosition(Constants.Arm.safe_pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dSpeed = m_speed.getAsDouble();
    m_elevator.setMotors(dSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
