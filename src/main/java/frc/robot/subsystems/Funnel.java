// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Funnel extends SubsystemBase {
  /** Creates a new Index. */
  private SparkMax motorWinch = new SparkMax(Constants.Funnel.winchID, MotorType.kBrushless);

  private RelativeEncoder encoderWinch;
  private SparkMaxConfig configWinch;

  public Funnel() {
    encoderWinch = motorWinch.getEncoder();
    configWinch = new SparkMaxConfig();
    configWinch.idleMode(IdleMode.kBrake);
    motorWinch.configure(
        configWinch, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoderWinch.setPosition(0); // improve later if time, possibly set when enabled
  }

  public void wind() {
    motorWinch.set(Constants.Funnel.winchSpeed);
  }

  public void unwind() {
    motorWinch.set(-Constants.Funnel.winchSpeed);
  }

  public void stop() {
    motorWinch.set(0);
  }

  public Boolean limitReached() {
    double position = encoderWinch.getPosition();
    if (position > Constants.Funnel.limit) {
      return true;
    } else if (position < -Constants.Funnel.limit) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (limitReached()) {
      stop();
    }
  }
}
