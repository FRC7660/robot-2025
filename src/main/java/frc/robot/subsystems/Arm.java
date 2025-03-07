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

  public Encoder encoderArm = new Encoder(1, 2);

  private double desiredSpeed = 0;
  private ArmFeedforward feedforward;

  private PIDController controller =
      new PIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd);

  public Arm() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      motorArmSim = new TalonFXSimState(motorArm);
    }

    motorArm.setPosition(0);

    motorArm.setNeutralMode(NeutralModeValue.Coast);
    motorArm.setInverted(true);

    feedforward = new ArmFeedforward(0, 0, 0);
    SmartDashboard.putNumber("Arm FF", 0);
    SmartDashboard.putNumber("Arm kS", 0);
    SmartDashboard.putNumber("Arm kG", Constants.Arm.kG);
    SmartDashboard.putNumber("Arm kV", 0);

    SmartDashboard.putNumber("Arm-Desired-Speed", desiredSpeed);
  }

  public void setMotor(double speed) {
    desiredSpeed = speed;
  }

  public void raise() {
    desiredSpeed = Constants.Arm.armSpeed;
  }

  public void stop() {
    desiredSpeed = 0;
  }

  public void setPosition(double position) {
    desiredSpeed =
        controller.calculate(motorArm.getPosition().getValueAsDouble(), position)
            + Math.cos(
                (encoderArm.get() - Constants.Arm.horizontalCounts)
                    / Constants.Arm.countsPerRadian);
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

  public double getPosition() {
    return motorArm.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm-Pos", motorArm.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm-Velo", motorArm.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Arm-Encoder", encoderArm.get());

    double posff =
        (Constants.Arm.motorOffset - motorArm.getPosition().getValueAsDouble())
            * Constants.Arm.radiansPerMotorRotation;
    SmartDashboard.putNumber("Arm PosFF", posff);

    // desiredSpeed = SmartDashboard.getNumber("Arm-Desired-Speed", 0);

    feedforward.setKg(Constants.Arm.kG);

    double outff = feedforward.calculate(posff, 0);

    SmartDashboard.putNumber("Arm FF", outff);

    // if (desiredSpeed < 0 && encoderArm.get() <= Constants.Arm.reverseLimit) {
    //   desiredSpeed = 0;
    // }
    // if (desiredSpeed > 0 && encoderArm.get() >= Constants.Arm.forewardLimit) {
    //   desiredSpeed = 0;
    // }
    double speed = MathUtil.applyDeadband(desiredSpeed, 0.01);
    motorArm.set(speed);
    SmartDashboard.putNumber("Arm desiredSpeed", speed);
  }

  public void simulationPeriodic() {
    motorArmSim.setSupplyVoltage(
        RobotController
            .getBatteryVoltage()); // need to fix sim capabilites, find talon version of iterate
    // function
  }

  public Command manualArm(DoubleSupplier speed) {
    return this.run(() -> setMotor(speed.getAsDouble() * 0.3));
  }
}
