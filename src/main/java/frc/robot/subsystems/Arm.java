// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public TalonFX motorArm = new TalonFX(Constants.Arm.motorID);

  private TalonFXSimState motorArmSim;

  private boolean manualMode = false;
  private boolean tuning = false;
  private boolean debug = false;

  public Encoder encoderArm = new Encoder(1, 2);

  private double desiredOutput = 0;
  private ArmFeedforward m_feedforward;

  private double targetPos;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(Constants.Arm.kMaxVelocity, Constants.Arm.kMaxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd, m_constraints, Constants.Arm.kDt);

  public Arm() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      motorArmSim = new TalonFXSimState(motorArm);
    }

    motorArm.setNeutralMode(NeutralModeValue.Coast);
    motorArm.setInverted(true);

    m_feedforward =
        new ArmFeedforward(
            Constants.Arm.kS, Constants.Arm.kGVoltage, Constants.Arm.kV, Constants.Arm.kA);
    SmartDashboard.putNumber("Arm FF", 0);
    SmartDashboard.putNumber("Arm kS", m_feedforward.getKs());
    SmartDashboard.putNumber("Arm kG", m_feedforward.getKg());
    SmartDashboard.putNumber("Arm kV", m_feedforward.getKv());
    SmartDashboard.putNumber("Arm kA", m_feedforward.getKa());

    SmartDashboard.putNumber("Arm P", m_controller.getP());
    SmartDashboard.putNumber("Arm I", m_controller.getI());
    SmartDashboard.putNumber("Arm D", m_controller.getD());

    SmartDashboard.putNumber("Arm-Max-Velo", Constants.Arm.kMaxVelocity);
    SmartDashboard.putNumber("Arm-Max-Acceleration", Constants.Arm.kMaxAcceleration);

    SmartDashboard.putBoolean("Arm-manualMode", manualMode);
  }

  public void setMotor(double output) {
    desiredOutput = output;
  }

  public void setTarget(double Pos) {
    targetPos = Pos;
    manualMode = false;
  }

  public double getTarget() {
    return targetPos;
  }

  public Boolean armAtExtended() {
    double position = motorArm.getPosition().getValueAsDouble();
    if (position > Constants.Arm.forewardLimit) {
      System.out.println("Upper Arm Limit Reached");
      return true;
    }
    return false;
  }

  public Boolean armAtZero() {
    double position = motorArm.getPosition().getValueAsDouble();
    if (position < Constants.Arm.zeroPos) {
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

  public boolean isInSafeZone() {
    return (getPosition() <= Constants.Arm.safePosIn && getPosition() >= Constants.Arm.safePosOut);
  }

  public void holdCurrentPosition() {
    targetPos = getPosition();
    manualMode = false;
  }

  public void manualIn() {
    manualMode = true;
    desiredOutput = Constants.Arm.armSpeed;
  }

  public double convertRevToRad(double pos) {
    return ((Constants.Arm.motorOffset - pos) * Constants.Arm.radiansPerMotorRotation);
  }

  public void manualOut() {
    manualMode = true;
    desiredOutput = -Constants.Arm.armSpeed;
  }

  private void setVoltage(double output) {
    double adjusted = output;
    if (output < 0 && armAtExtended()) {
      adjusted =
          m_feedforward.calculate(
              convertRevToRad(Constants.Arm.forewardLimit), m_controller.getSetpoint().velocity);
    } else if (output > 0 && armAtZero()) {
      adjusted = 0;
    }
    motorArm.setVoltage(adjusted);
  }

  @Override
  public void periodic() {
    if (manualMode) {
      motorArm.set(desiredOutput);
      return;
    }

    if (debug) {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Arm-Pos", motorArm.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Arm-Velo", motorArm.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Arm-Encoder", encoderArm.get());

      SmartDashboard.putNumber("Arm-setpoint", m_controller.getSetpoint().position);
    }

    if (tuning) {
      m_controller.setP(SmartDashboard.getNumber("Arm P", Constants.Arm.kp));
      m_controller.setI(SmartDashboard.getNumber("Arm I", Constants.Arm.ki));
      m_controller.setD(SmartDashboard.getNumber("Arm D", Constants.Arm.kd));

      m_feedforward.setKg(SmartDashboard.getNumber("Arm kG", Constants.Arm.kGVoltage));
      m_feedforward.setKs(SmartDashboard.getNumber("Arm kS", Constants.Arm.kS));
      m_feedforward.setKv(SmartDashboard.getNumber("Arm kV", Constants.Arm.kV));
      m_feedforward.setKa(SmartDashboard.getNumber("Arm kA", Constants.Arm.kA));

      m_controller.setConstraints(
          new TrapezoidProfile.Constraints(
              SmartDashboard.getNumber("Arm-Max-Velo", Constants.Arm.kMaxVelocity),
              SmartDashboard.getNumber("Arm-Max-Acceleration", Constants.Arm.kMaxAcceleration)));
    }

    m_controller.setGoal(targetPos);

    double outff =
        m_feedforward.calculate(
            convertRevToRad(motorArm.getPosition().getValueAsDouble()),
            m_controller.getSetpoint().velocity);
    double outPID = m_controller.calculate(getPosition());

    if (debug) {
      SmartDashboard.putNumber("Arm-PID", outPID);
      SmartDashboard.putNumber(
          "Arm-PosFF", convertRevToRad(motorArm.getPosition().getValueAsDouble()));
      SmartDashboard.putNumber("Arm-FF", outff);
    }

    setVoltage(outPID + outff);
  }

  public void simulationPeriodic() {
    motorArmSim.setSupplyVoltage(
        RobotController
            .getBatteryVoltage()); // need to fix sim capabilites, find talon version of iterate
    // function
  }
}
