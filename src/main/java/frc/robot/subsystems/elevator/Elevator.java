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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorState;

// import com.revrobotics.CANSparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {

  public final SparkMax motorAlpha =
      new SparkMax(Constants.Elevator.motorAlphaID, MotorType.kBrushless);
  public final SparkMax motorBeta =
      new SparkMax(Constants.Elevator.motorBetaID, MotorType.kBrushless);
  public int alphaInversion = -1; // The factor by which a motor's rotation should be applied
  public int betaInversion = -1;
  public DigitalInput bottomLimit = new DigitalInput(Constants.Elevator.lowerlimitID);
  public DigitalInput topLimit = new DigitalInput(10);
  public SparkMaxConfig alphaConfig = new SparkMaxConfig();
  public SparkMaxConfig betaConfig = new SparkMaxConfig();

  private RelativeEncoder motorAlphaEncoder = motorAlpha.getEncoder();
  private RelativeEncoder motorBetaEncoder = motorBeta.getEncoder();

  private SparkMaxSim motorSim;
  private SparkRelativeEncoderSim motorSimEncoder;

  private PIDController elevatorPid =
      new PIDController(Constants.elevatorP, Constants.elevatorI, Constants.elevatorD);

  public void raise() {
    motorAlpha.set(.5 * alphaInversion);
  }

  public void lower() {
    motorAlpha.set(-.5 * alphaInversion);
  }

  public void setMotors(double speed, boolean overrideInversion) {
    Double speedMulti;
    if (speed < 0) {
      speedMulti = 0.25;
    } else {
      speedMulti = 1.0;
    }
    Double outputSpeed = speed * speedMulti;
    motorAlpha.set(outputSpeed);
    motorBeta.set(outputSpeed);
    SmartDashboard.putNumber("Elevator Output", outputSpeed);
    SmartDashboard.putNumber("Motor Alpha Speed", motorAlpha.get());
    SmartDashboard.putNumber("Motor Beta Speed", motorBeta.get());
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
    Double output;
    System.out.println("state is " + state);
    switch (state) {
      case L1:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(), Constants.l1height);
        break;
      case L2:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(), Constants.l2height);
        break;
      case L3:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(), Constants.l3height);
        break;
      case L4:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(), Constants.l4height);
        break;
      case ZERO:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(), Constants.zeroHeight);
        break;

      default:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(), Constants.l1height);
        break;
    }
    setMotors(output * 0.1, true);
  }

  public Elevator() {
    // Clockwise - up, Counter - down, same for both motors
    // Gear ratio - 6:1
    // motorSim.setPosition(5);

    motorAlphaEncoder.setPosition(0);
    System.out.println("Motor Position:" + motorAlphaEncoder.getPosition());

    alphaConfig.softLimit.forwardSoftLimit(Constants.Elevator.upperLimit);
    alphaConfig.softLimit.reverseSoftLimit(Constants.Elevator.lowerLimit);
    alphaConfig.softLimit.forwardSoftLimitEnabled(true);
    alphaConfig.softLimit.reverseSoftLimitEnabled(true);
    alphaConfig.idleMode(IdleMode.kBrake);
    alphaConfig.inverted(false);
    betaConfig.apply(alphaConfig);

    motorAlpha.configure(
        alphaConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    motorBeta.configure(
        betaConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    if (Constants.currentMode == Constants.Mode.SIM) {
      motorSim = new SparkMaxSim(motorAlpha, DCMotor.getNeo550(1));
      motorSimEncoder = motorSim.getRelativeEncoderSim();
    }
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Motor Alpha Speed", motorAlpha.get());
    SmartDashboard.putNumber("Motor Alpha Position", motorAlphaEncoder.getPosition());
    SmartDashboard.putBoolean("Elevator Limit Reached", !bottomLimit.get());
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
