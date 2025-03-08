// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorState;

// import com.revrobotics.CANSparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {

  private Integer counter = 0;

  public final SparkFlex motorAlpha =
      new SparkFlex(Constants.Elevator.motorAlphaID, MotorType.kBrushless);
  public final SparkFlex motorBeta =
      new SparkFlex(Constants.Elevator.motorBetaID, MotorType.kBrushless);
  public int alphaInversion = -1; // The factor by which a motor's rotation should be applied
  public int betaInversion = -1;
  public DigitalInput bottomLimit = new DigitalInput(Constants.Elevator.lowerlimitID);
  // public DigitalInput topLimit = new DigitalInput(10);
  public SparkFlexConfig alphaConfig = new SparkFlexConfig();
  public SparkFlexConfig betaConfig = new SparkFlexConfig();
  public Double targetPosition = 0.0;

  private RelativeEncoder motorAlphaEncoder = motorAlpha.getEncoder();
  private RelativeEncoder motorBetaEncoder = motorBeta.getEncoder();

  private SparkMaxSim motorSim;
  private SparkRelativeEncoderSim motorSimEncoder;

  // private PIDController elevatorPid =
  //     new PIDController(Constants.elevatorP, Constants.elevatorI, Constants.elevatorD);

  Double eKp = 0.0;
  Double eKi = 0.0;
  Double eKd = 0.0;
  Double eKs = 0.0;
  Double eKg = Constants.Elevator.feedForward;
  Double eKv = 0.0;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(0.4, 0.2);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(eKp, eKi, eKd, m_constraints, 0.02);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(eKs, eKg, eKv);

  // public void raise() {
  //   motorAlpha.set(.5 * alphaInversion);
  // }

  // public void lower() {
  //   motorAlpha.set(-.5 * alphaInversion);
  // }

  public boolean isAtBottom() {
    return (motorAlphaEncoder.getPosition() < Constants.Elevator.lowerLimit || !bottomLimit.get());
  }

  public boolean isAtTop() {
    return (motorAlphaEncoder.getPosition() > Constants.Elevator.upperLimit);
  }

  public void setMotors(double speed) {
    Double speedMulti;
    if (speed < 0) {
      speedMulti = 0.2;
    } else {
      speedMulti = 0.5;
    }
    Double outputSpeed = speed * speedMulti;
    if (outputSpeed < 0 && isAtBottom()) {
      motorAlpha.set(0);
      motorBeta.set(0);
    } else if (outputSpeed > 0 && isAtTop()) {
      motorAlpha.set(Constants.Elevator.feedForward);
      motorBeta.set(Constants.Elevator.feedForward);
    } else {
      motorAlpha.set(outputSpeed);
      motorBeta.set(outputSpeed);
    }
    SmartDashboard.putNumber("Elevator Output", outputSpeed);
    SmartDashboard.putNumber("Motor Alpha Speed", motorAlpha.get());
    SmartDashboard.putNumber("Motor Beta Speed", motorBeta.get());
  }

  private void setCalculatedMotors(Double output, Double feedForward) {
    motorAlpha.set(output + feedForward);
    motorBeta.set(output + feedForward);
  }

  public Double getHeight() {
    // Height gained from one rotation: 0.91978999in * 2
    // Sprocket Diameter: 1.75667in
    // Circumfrence: 5.518737in
    Double height;
    Double baseRotations = motorAlphaEncoder.getPosition();
    height = baseRotations * 0.91979 * 2;
    return height;
  }

  public void setState(ElevatorState state) {
    counter += 1;
    Double output;
    Double goal;
    Double feedForward;
    System.out.println("state is " + state);
    switch (state) {
      case L1:
        goal = Constants.l1height;
        break;
      case L2:
        goal = Constants.l2height;
        break;
      case L3:
        goal = Constants.l3height;
        break;
      case L4:
        goal = Constants.l4height;
        break;
      case ZERO:
        goal = Constants.zeroHeight;
        break;

      default:
        goal = 0.0;
        break;
    }

    m_controller.setGoal(goal);
    output = m_controller.calculate(motorAlphaEncoder.getPosition());
    feedForward = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    SmartDashboard.putNumber("PID Output", output);
    SmartDashboard.putNumber("PID feedForward calculation", feedForward);
    SmartDashboard.putNumber("PID Goal", m_controller.getGoal().position);
    SmartDashboard.putNumber("Elevator Counter", counter);
    setCalculatedMotors(output, feedForward);
  }

  public Elevator() {
    // Clockwise - up, Counter - down, same for both motors
    // Gear ratio - 6:1
    // motorSim.setPosition(5);

    SmartDashboard.putNumber("eKp", eKp);
    SmartDashboard.putNumber("eKi", eKi);
    SmartDashboard.putNumber("eKd", eKd);
    SmartDashboard.putNumber("eKs", eKs);
    SmartDashboard.putNumber("eKg", eKg);
    SmartDashboard.putNumber("eKv", eKv);

    motorAlphaEncoder.setPosition(0);
    System.out.println("Motor Position:" + motorAlphaEncoder.getPosition());

    alphaConfig.softLimit.forwardSoftLimit(Constants.Elevator.upperLimit);
    alphaConfig.softLimit.reverseSoftLimit(Constants.Elevator.lowerLimit);
    alphaConfig.softLimit.forwardSoftLimitEnabled(true);
    alphaConfig.softLimit.reverseSoftLimitEnabled(true);
    alphaConfig.idleMode(IdleMode.kBrake);
    alphaConfig.inverted(true);
    betaConfig.apply(alphaConfig);

    motorAlpha.configure(
        alphaConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    motorBeta.configure(
        betaConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    if (Constants.currentMode == Constants.Mode.SIM) {
      // motorSim = new SparkFlexSim(motorAlpha, DCMotor.getNeo550(1));
      motorSimEncoder = motorSim.getRelativeEncoderSim();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Alpha Speed", motorAlpha.get());
    SmartDashboard.putNumber("Motor Alpha Position", motorAlphaEncoder.getPosition());
    SmartDashboard.putBoolean("Elevator Limit Reached", !bottomLimit.get());
    SmartDashboard.putNumber("Alpha Applied", motorAlpha.getAppliedOutput());

    m_controller.setPID(
        SmartDashboard.getNumber("eKp", 0),
        SmartDashboard.getNumber("eKi", 0),
        SmartDashboard.getNumber("eKd", 0));

    m_feedforward.setKs(SmartDashboard.getNumber("eKs", 0));
    m_feedforward.setKg(SmartDashboard.getNumber("eKg", 0));
    m_feedforward.setKv(SmartDashboard.getNumber("eKv", 0));
  }

  public void simulationPeriodic() {
    double velo = motorAlpha.get() * 15;
    double voltage = RoboRioSim.getVInVoltage();
    if (motorSim.getPosition() + velo < Constants.Elevator.lowerLimit
        || motorSim.getPosition() + velo > Constants.Elevator.upperLimit) {
      motorAlpha.set(0);
      velo = 0;
    }
    motorSim.iterate(velo, voltage, Constants.simCycleTime);
    SmartDashboard.putNumber("Motor Sim Position", motorSim.getRelativeEncoderSim().getPosition());
    SmartDashboard.putNumber("Sim Velo", velo);
    SmartDashboard.putNumber("Sim Voltage", voltage);
  }
}
