// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class releaseCoral extends Command {
  /** Creates a new releaseCoral. */
  private final Claw claw;
  

  public releaseCoral(Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(claw.getClawSensorHit()){ //need to clarify/fix to return correct boolean!
    System.out.println("Coral is released");
    claw.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(claw.getClawSensorHit()){ //need to clarify/fix to return correct boolean!
      claw.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return claw.getClawSensorHit(); //need to make this look for false instead of true!
  }
}
