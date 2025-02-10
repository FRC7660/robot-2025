// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private SparkMax motorClimb = new SparkMax(Constants.Climb.MotorClimbID, MotorType.kBrushless);

  private RelativeEncoder encoderClimb;
  private SparkMaxConfig configClimb;

  private SparkMaxSim motorClimbSim;

  public Climb() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      motorClimbSim = new SparkMaxSim(motorClimb, DCMotor.getNEO(1));
    }

    encoderClimb = motorClimb.getEncoder();
    configClimb = new SparkMaxConfig();
    configClimb.idleMode(IdleMode.kBrake);
    motorClimb.configure(
        configClimb, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoderClimb.setPosition(0);
  }

  public Boolean climbFinished() {
    double position = encoderClimb.getPosition();
    if (position > Constants.Climb.climbEncoderLimit) {
      System.out.println("Limit Reached");
      return true;
    }
    return false;
  }

  public void lower() {
    motorClimb.set(Constants.Climb.climbSpeed);
  }

  public void stop() {
    motorClimb.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (climbFinished()) {
      motorClimb.set(0);
    }

    SmartDashboard.putNumber("Climb-Pos", encoderClimb.getPosition());
  }

  public void simulationPeriodic() {
    double velo = motorClimb.get() * 1.0;
    double voltage = RoboRioSim.getVInVoltage();
    motorClimbSim.iterate(velo, voltage, Constants.simCycleTime);
  }
}
