// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmPIDTest extends Command {
  private PIDController pid;
  private ArmFeedforward feedforward;
  private final Arm arm;

  /** Creates a new ArmPIDTest. */
  public ArmPIDTest(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    pid = new PIDController(0, 0, 0);
    feedforward = new ArmFeedforward(0, 0, 0);
    double posff =
        (Constants.Arm.motorOffset - arm.getPosition()) * Constants.Arm.radiansPerMotorRotation;

    SmartDashboard.putNumber("Arm P", 0);
    SmartDashboard.putNumber("Arm I", 0);
    SmartDashboard.putNumber("Arm D", 0);
    SmartDashboard.putNumber("Arm-Test-Pos", 0);
    SmartDashboard.putNumber("Arm Test Velo", 0);
    SmartDashboard.putNumber("Arm PosFF", posff);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.setP(SmartDashboard.getNumber("Arm P", 0.01));
    pid.setI(SmartDashboard.getNumber("Arm I", 0.005));
    pid.setD(SmartDashboard.getNumber("Arm D", 0));

    SmartDashboard.getNumber("Arm-Test-Pos", 0);
    double outPID = pid.calculate(arm.getPosition(),
     SmartDashboard.getNumber("Arm-Test-Pos", 0));
    double posff = SmartDashboard.getNumber("Arm PosFF", 0);
    double desiredSpeed = SmartDashboard.getNumber("Arm-Desired-Speed", 0);
  
    SmartDashboard.putNumber("Arm outPID", outPID);

    feedforward.setKa(SmartDashboard.getNumber("Arm kS", 0));
    feedforward.setKg(SmartDashboard.getNumber("Arm kG", Constants.Arm.kG));
    feedforward.setKv(SmartDashboard.getNumber("Arm kV", 0));
    double outff = feedforward.calculate(posff, desiredSpeed);
    arm.setMotor(MathUtil.clamp(outff + outPID, -0.4, 0.4));

    double measured_angle_rad =
        (arm.getPosition() - Constants.Arm.horizontalCounts) / Constants.Arm.countsPerRadian;

    SmartDashboard.putNumber("Arm Meas Angle Radians", measured_angle_rad);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
