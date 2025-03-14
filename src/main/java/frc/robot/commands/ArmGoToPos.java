// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmGoToPos extends Command {
  private final Arm m_arm;
  private final Elevator m_elevator;
  private final double targetPos;
  private double limitOut, limitIn;

  /** Creates a new ArmGoToPos. */
  public ArmGoToPos(Arm arm, Elevator elevator, double Pos) { // ADD ELEVATOR TO REQUIREMENTS
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    targetPos = Pos;
    m_elevator = elevator;
    addRequirements(m_arm, m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.hold();

    if (!m_elevator.isAtBottom()) {
      limitIn = Constants.Arm.safePosIn;
      limitOut = Constants.Arm.safePosOut;
    } else {
      limitIn = Constants.Arm.zeroPos;
      limitOut = Constants.Arm.forewardLimit;
    }

    if (targetPos > limitIn) {
      cancel();
    }

    if (targetPos < limitOut) {
      cancel();
    }

    m_arm.setTarget(targetPos);
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
    return m_arm.atTargetPos();
  }
}
