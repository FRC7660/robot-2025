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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private SparkMax motorArm = new SparkMax(Constants.Arm.motorArmID, MotorType.kBrushless);
  private RelativeEncoder encoderArm;
  private SparkMaxConfig configArm;

  private SparkMaxSim motorArmSim;

  public Arm() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      motorArmSim = new SparkMaxSim(motorArm, DCMotor.getNEO(1));
    }

    encoderArm = motorArm.getEncoder();
    configArm = new SparkMaxConfig();
    configArm.idleMode(IdleMode.kBrake);
    motorArm.configure(
        configArm, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoderArm.setPosition(0);
  }

  public void raise(){
    motorArm.set(Constants.Arm.armSpeed);
  }

  public void stop(){
    motorArm.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm-Pos", encoderArm.getPosition());
    SmartDashboard.putNumber("Arm-Velo", encoderArm.getVelocity());
  }
}
