package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestAuto extends Command {
  /** Creates a new Test. */
  private String name;


  public TestAuto(String nameInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    name = nameInput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(name + " is Initialized");
    //SmartDashboard.putNumber(getName(), 2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(name + " is Executing");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(name + " is not Finished");
    return true;
  }
}