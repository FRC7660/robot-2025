// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final SparkFlex motor = new SparkFlex(Constants.Intake.motorID, MotorType.kBrushless);
  private final SparkFlexConfig motorConfig = new SparkFlexConfig();
  private final RelativeEncoder motorEncoder = motor.getEncoder();

  private SparkFlexSim motorSim;
  private SparkRelativeEncoderSim motorSimEncoder;

  // PID and Feedforward parameters
  private double kp = Constants.Intake.kp;
  private double ki = Constants.Intake.ki;
  private double kd = Constants.Intake.kd;
  private double kS = Constants.Intake.kS;
  private double kV = Constants.Intake.kV;
  private double maxVelocity = Constants.Intake.kMaxVelocity;
  private double maxAcceleration = Constants.Intake.kMaxAcceleration;

  private double manualOutput = 0.0;

  private boolean debug = false;
  private boolean tuning = false;
  private boolean manual = false;

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(kp, ki, kd, constraints, Constants.Intake.kDt);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

  public Intake() {
    // Initialize SmartDashboard values for tuning
    SmartDashboard.putNumber("Intake kp", kp);
    SmartDashboard.putNumber("Intake ki", ki);
    SmartDashboard.putNumber("Intake kd", kd);
    SmartDashboard.putNumber("Intake kS", kS);
    SmartDashboard.putNumber("Intake kV", kV);
    SmartDashboard.putNumber("Intake MaxVel", maxVelocity);
    SmartDashboard.putNumber("Intake MaxAccel", maxAcceleration);

    // Reset encoder position
    motorEncoder.setPosition(0);
    System.out.println("Intake Motor Position: " + getPosition());

    // Configure motor with soft limits
    motorConfig.softLimit.forwardSoftLimit(Constants.Intake.upperLimit);
    motorConfig.softLimit.reverseSoftLimit(Constants.Intake.lowerLimit);
    motorConfig.softLimit.forwardSoftLimitEnabled(true);
    motorConfig.softLimit.reverseSoftLimitEnabled(true);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Simulation setup
    if (Constants.currentMode == Constants.Mode.SIM) {
      motorSim = new SparkFlexSim(motor, DCMotor.getNeoVortex(1));
      motorSimEncoder = motorSim.getRelativeEncoderSim();
    }
  }

  /**
   * Get current position in rotations
   *
   * @return Position in rotations
   */
  public double getPosition() {
    return motorEncoder.getPosition();
  }

  /**
   * Get current velocity in rotations per second
   *
   * @return Velocity in rotations per second
   */
  public double getVelocity() {
    return motorEncoder.getVelocity() / 60.0; // Convert RPM to rotations per second
  }

  /**
   * Check if at lower limit
   *
   * @return true if at or below lower limit
   */
  public boolean isAtLowerLimit() {
    return getPosition() <= Constants.Intake.lowerLimit;
  }

  /**
   * Check if at upper limit
   *
   * @return true if at or above upper limit
   */
  public boolean isAtUpperLimit() {
    return getPosition() >= Constants.Intake.upperLimit;
  }

  /**
   * Set voltage to motor with limit checking
   *
   * @param voltage Voltage to apply
   */
  private void setVoltage(double voltage) {
    double adjusted = voltage;
    if (voltage < 0 && isAtLowerLimit()) {
      adjusted = 0;
    } else if (voltage > 0 && isAtUpperLimit()) {
      adjusted = 0;
    }
    motor.setVoltage(adjusted);
  }

  /**
   * Set target position for position control
   *
   * @param position Target position in rotations
   */
  public void setTarget(double position) {
    controller.reset(getPosition());
    controller.setGoal(position);
    manual = false;
  }

  /**
   * Check if at goal position
   *
   * @return true if within tolerance of goal
   */
  public boolean isAtGoal() {
    return MathUtil.isNear(controller.getGoal().position, getPosition(), 0.5);
  }

  /**
   * Hold current position
   */
  public void hold() {
    controller.reset(getPosition());
    controller.setGoal(getPosition());
    manual = false;
  }

  /**
   * Manual control forward
   */
  public void manualForward() {
    manual = true;
    manualOutput = Constants.Intake.manualSpeed * 12; // Convert to voltage
    controller.reset(getPosition());
  }

  /**
   * Manual control reverse
   */
  public void manualReverse() {
    manual = true;
    manualOutput = -Constants.Intake.manualSpeed * 12; // Convert to voltage
    controller.reset(getPosition());
  }

  /**
   * Stop manual control
   */
  public void stopManual() {
    manual = false;
    hold();
  }

  @Override
  public void periodic() {
    // Update SmartDashboard
    SmartDashboard.putNumber("Intake Speed", motor.get());
    SmartDashboard.putNumber("Intake Position", getPosition());
    SmartDashboard.putNumber("Intake Velocity", getVelocity());

    if (debug) {
      SmartDashboard.putNumber("Intake Applied Output", motor.getAppliedOutput());
      SmartDashboard.putBoolean("Intake At Lower Limit", isAtLowerLimit());
      SmartDashboard.putBoolean("Intake At Upper Limit", isAtUpperLimit());
    }

    // Tuning mode - update PID and feedforward from SmartDashboard
    if (tuning) {
      controller.setPID(
          SmartDashboard.getNumber("Intake kp", kp),
          SmartDashboard.getNumber("Intake ki", ki),
          SmartDashboard.getNumber("Intake kd", kd));

      feedforward.setKs(SmartDashboard.getNumber("Intake kS", kS));
      feedforward.setKv(SmartDashboard.getNumber("Intake kV", kV));

      controller.setConstraints(
          new TrapezoidProfile.Constraints(
              SmartDashboard.getNumber("Intake MaxVel", maxVelocity),
              SmartDashboard.getNumber("Intake MaxAccel", maxAcceleration)));

      SmartDashboard.putNumber("Intake Setpoint", controller.getSetpoint().position);
    }

    // Calculate PID output
    double pidOutput = controller.calculate(getPosition());
    double ffOutput = feedforward.calculate(controller.getSetpoint().velocity);

    SmartDashboard.putNumber("Intake PID Output", pidOutput);
    SmartDashboard.putNumber("Intake FF Output", ffOutput);
    SmartDashboard.putNumber("Intake Goal", controller.getGoal().position);

    // Apply control
    if (manual) {
      setVoltage(manualOutput);
    } else {
      setVoltage(pidOutput + ffOutput);
    }
  }

  @Override
  public void simulationPeriodic() {
    double velocity = motor.get() * maxVelocity;
    double voltage = RoboRioSim.getVInVoltage();

    // Limit simulation position
    if (motorSim.getPosition() + velocity * Constants.simCycleTime < Constants.Intake.lowerLimit
        || motorSim.getPosition() + velocity * Constants.simCycleTime
            > Constants.Intake.upperLimit) {
      motor.set(0);
      velocity = 0;
    }

    motorSim.iterate(velocity, voltage, Constants.simCycleTime);
    SmartDashboard.putNumber("Intake Sim Position", motorSim.getRelativeEncoderSim().getPosition());
    SmartDashboard.putNumber("Intake Sim Velocity", velocity);
  }

  /**
   * Enable/disable debug mode
   *
   * @param enabled true to enable debug output
   */
  public void setDebug(boolean enabled) {
    debug = enabled;
  }

  /**
   * Enable/disable tuning mode
   *
   * @param enabled true to enable tuning mode
   */
  public void setTuning(boolean enabled) {
    tuning = enabled;
  }
}
