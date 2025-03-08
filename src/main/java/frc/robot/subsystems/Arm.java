// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public TalonFX motorArm = new TalonFX(Constants.Arm.motorID);

  private TalonFXSimState motorArmSim;

  private boolean manualMode = false;

  public Encoder encoderArm = new Encoder(1, 2);

  private double desiredOutput = 0;
  private ArmFeedforward feedforward;

  private double targetPos;

  private PIDController pid =
      new PIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd);

  public Arm() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      motorArmSim = new TalonFXSimState(motorArm);
    }

    motorArm.setNeutralMode(NeutralModeValue.Coast);
    motorArm.setInverted(true);

    feedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);
    SmartDashboard.putNumber("Arm FF", 0);
    SmartDashboard.putNumber("Arm kS", feedforward.getKs());
    SmartDashboard.putNumber("Arm kG", feedforward.getKg());
    SmartDashboard.putNumber("Arm kV", feedforward.getKv());
    
    SmartDashboard.putNumber("Arm P", pid.getP());
    SmartDashboard.putNumber("Arm I", pid.getI());
    SmartDashboard.putNumber("Arm D", pid.getD());

    SmartDashboard.putBoolean("Arm-manualMode", manualMode);
  }

  public void setMotor(double output) {
    desiredOutput = output;
  }

  public void setManual(boolean manual){
    manualMode = manual;
  }

  public void setTarget(double Pos) {
    targetPos = Pos;
  }

  public double getTarget() {
    return targetPos;
  }

  public Boolean armAtMax() {
    double position = motorArm.getPosition().getValueAsDouble();
    if (position > Constants.Arm.armEncoderUpperLimit) {
      System.out.println("Upper Arm Limit Reached");
      return true;
    }
    return false;
  }

  public Boolean armAtMin() {
    double position = motorArm.getPosition().getValueAsDouble();
    if (position < Constants.Arm.armEncoderLowerLimit) {
      System.out.println("Lower Arm Limit Reached");
      return true;
    }
    return false;
  }

  public boolean atTargetPos() {
    return MathUtil.isNear(targetPos, getPosition(), Constants.Arm.endDeadband);
  }

  public double getPosition() {
    return motorArm.getPosition().getValueAsDouble();
  }

  public void holdCurrentPosition() {
    targetPos = getPosition();
  }

  @Override
  public void periodic() {
    if(manualMode){
      motorArm.set(desiredOutput);
      return;
    }
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm-Pos", motorArm.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm-Velo", motorArm.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Arm-Encoder", encoderArm.get());

    double posff =
        (Constants.Arm.motorOffset - motorArm.getPosition().getValueAsDouble())
            * Constants.Arm.radiansPerMotorRotation;
    SmartDashboard.putNumber("Arm-PosFF", posff);

    double outff = feedforward.calculate(posff, 0);

    SmartDashboard.putNumber("Arm FF", outff);

    double outPID = pid.calculate(getPosition(), targetPos);

    motorArm.set(MathUtil.clamp(outff + outPID, -0.4, 0.4));
  }

  public void simulationPeriodic() {
    motorArmSim.setSupplyVoltage(
        RobotController
            .getBatteryVoltage()); // need to fix sim capabilites, find talon version of iterate
    // function
  }

  public Command manualArm(DoubleSupplier speed) {
    return this.runEnd(() -> setMotor(speed.getAsDouble() * 0.3), () -> holdCurrentPosition());
  }
}
