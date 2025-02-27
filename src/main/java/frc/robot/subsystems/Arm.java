// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
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
  private TalonFX motorArm = new TalonFX(Constants.Arm.motorID);

  private TalonFXSimState motorArmSim;

  public Encoder encoderArm = new Encoder(1, 2);

  private double desiredSpeed = 0;

  private PIDController controller =
      new PIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd);

  public Arm() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      motorArmSim = new TalonFXSimState(motorArm);
    }

    motorArm.setPosition(0);

    motorArm.setNeutralMode(NeutralModeValue.Brake);
    motorArm.setInverted(true);
  }

  public void setMotor(double speed) {
    desiredSpeed = speed * 0.5;
  }

  public void raise() {
    desiredSpeed = Constants.Arm.armSpeed;
  }

  public void stop() {
    desiredSpeed = 0;
  }

  public void setPosition(double position) {
    desiredSpeed = controller.calculate(encoderArm.get(), position) + Math.sin((encoderArm.get() - Constants.Arm.verticleCounts)/Constants.Arm.countsPerRadian);
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
    return encoderArm.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm-Pos", motorArm.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm-Velo", motorArm.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Arm-Encoder", encoderArm.get());
    SmartDashboard.putNumber("Arm-Desired-Speed", desiredSpeed);

    if (desiredSpeed < 0 && encoderArm.get() <= Constants.Arm.reverseLimit) {
      desiredSpeed = 0;
    }
    if (desiredSpeed > 0 && encoderArm.get() >= Constants.Arm.forewardLimit) {
      desiredSpeed = 0;
    }
    desiredSpeed = MathUtil.applyDeadband(desiredSpeed, 0.05);
    motorArm.set(desiredSpeed);
  }

  public void simulationPeriodic() {
    motorArmSim.setSupplyVoltage(
        RobotController
            .getBatteryVoltage()); // need to fix sim capabilites, find talon version of iterate
    // function
  }

  public Command manualArm(DoubleSupplier speed) {
    return this.run(() -> setMotor(speed.getAsDouble()));
  }
}
