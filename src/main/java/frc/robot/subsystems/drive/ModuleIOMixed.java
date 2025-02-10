// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOMixed implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final TalonFX driveTalon;
  private final SparkMax turnSpark;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder cancoder;
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final PIDController turnPID;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  private final int moduleNumber;
  private Rotation2d rotation;

  public ModuleIOMixed(int module) {
    moduleNumber = module;
    rotation = new Rotation2d();
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> new Rotation2d();
        };
    driveTalon =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            });
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    cancoder =
        new CANcoder(
            switch (module) {
              case 0 -> frontLeftEncoderID;
              case 1 -> frontRightEncoderID;
              case 2 -> backLeftEncoderID;
              case 3 -> backRightEncoderID;
              default -> 0;
            });

    turnRelativeEncoder = turnSpark.getEncoder();

    turnPID = new PIDController(turnKp, turnKi, turnKd);
    turnPID.enableContinuousInput(
        turnPIDMinInput, turnPIDMaxInput); // Ensure continuous input is enabled
    turnPID.setTolerance(10);

    // Configure drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = new Slot0Configs().withKP(0.1).withKV(0.124);
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveEncoderPositionFactor;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.slipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DriveConstants.slipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.slipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(turnEncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pidf(turnKp, 0.0, turnKd, 0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    turnRelativeEncoder.setPosition(0.0);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());
    turnAbsolutePosition = cancoder.getAbsolutePosition();

    // Create odometry queues
    turnSpark
        .getEncoder()
        .setPosition(
            cancoder.getPosition().getValueAsDouble() * DriveConstants.turnEncoderPositionFactor);
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();

    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(turnSpark, cancoder.getAbsolutePosition()::getValueAsDouble);

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Configure periodic frames
    // BaseStatusSignal.setUpdateFrequencyForAll(Drive.ODOMETRY_FREQUENCY, drivePosition,
    // turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, driveVelocity, driveAppliedVolts, driveCurrent, turnAbsolutePosition);

    ParentDevice.optimizeBusUtilizationForAll(driveTalon);
  }

  public double getTurnPosition() {
    double turnPosition =
        MathUtil.inputModulus(
            (cancoder.getAbsolutePosition().getValueAsDouble()) * turnEncoderPositionFactor,
            turnPIDMinInput,
            turnPIDMaxInput);
    turnPosition %= 2 * Math.PI;
    return turnPosition;
  }

  public double getTurnVelocity() {
    double turnVelocity = (turnSpark.getEncoder().getVelocity()) / turnEncoderVelocityFactor;
    return turnVelocity;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        () ->
            MathUtil.inputModulus(
                (cancoder.getAbsolutePosition().getValueAsDouble()) * 2 * Math.PI,
                turnPIDMinInput,
                turnPIDMaxInput),
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(
        turnSpark,
        cancoder.getVelocity()::getValueAsDouble,
        (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    turnAbsolutePosition.refresh();
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / turnEncoderVelocityFactor;
    inputs.turnAppliedVolts = turnSpark.getAppliedOutput() * turnSpark.getBusVoltage();
    inputs.turnCurrentAmps = turnSpark.getOutputCurrent();

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (DriveConstants.driveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(
        switch (DriveConstants.driveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    this.rotation = rotation;
    double absolutePosition =
        MathUtil.inputModulus(
            (cancoder.getAbsolutePosition().getValueAsDouble()) * 2 * Math.PI,
            turnPIDMinInput,
            turnPIDMaxInput);
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnPID.setP(turnKp);
    turnPID.setI(turnKi);
    turnPID.setD(turnKd);
    turnPID.setTolerance(0.1, 0.1);
    double output = turnPID.calculate(setpoint, absolutePosition);
    output = MathUtil.clamp(output, -1, 1);
    turnSpark.set(output);
    if (moduleNumber == 0) {
      SmartDashboard.putNumber("Output", output);
    }
  }

  @Override
  public void putMyNumbers() {
    double absolutePosition =
        MathUtil.inputModulus(
            (cancoder.getAbsolutePosition().getValueAsDouble()) * 2 * Math.PI,
            turnPIDMinInput,
            turnPIDMaxInput);
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    SmartDashboard.putNumber("Absolute Position" + moduleNumber, absolutePosition);
    if (moduleNumber == 0) {
      SmartDashboard.putNumber("Setpoint", setpoint);
      SmartDashboard.putNumber("Actual", absolutePosition);
      SmartDashboard.putNumber("Setpoint pre-Mod", rotation.plus(zeroRotation).getRadians());
    }
  }
}
