// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private SparkMax motorClaw = new SparkMax(Constants.Claw.motorClawID, MotorType.kBrushless);

  private RelativeEncoder encoderClaw;
  private SparkMaxConfig configClaw;

  private SparkMaxSim motorClawSim;

  private DigitalInput clawBreakBeam = new DigitalInput(Constants.Claw.clawBeam);

  public Claw() {}

  public void start(){
    motorClaw.set(Constants.Claw.clawSpeed);
  }

  public void stop(){
    motorClaw.set(0);
  }

  public boolean getClawSensorHit() {
    return !clawBreakBeam.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Claw Break-Beam", getClawSensorHit());
  }
}
