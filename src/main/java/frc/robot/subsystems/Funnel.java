// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Funnel extends SubsystemBase {
  /** Creates a new Index. */
  private SparkFlex motorWinch = new SparkFlex(Constants.Funnel.winchID, MotorType.kBrushless);

  private RelativeEncoder encoderWinch;
  private SparkFlexConfig configWinch;

  // private SparkMaxSim motorWinchSim;

  private DigitalInput funnelLimit;

  private double desiredSpeed = 0;
  private PIDController pid = new PIDController(0, 0, 0);

  public Funnel() {
    // if (Constants.currentMode == Constants.Mode.SIM) {
    //   motorWinchSim = new SparkMaxSim(motorWinch, DCMotor.getNEO(1));
    // }
    encoderWinch = motorWinch.getEncoder();
    configWinch = new SparkFlexConfig();
    configWinch.idleMode(IdleMode.kBrake);
    motorWinch.configure(
        configWinch, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    encoderWinch.setPosition(0); // improve later if time, possibly set when enabled

    funnelLimit = new DigitalInput(Constants.Funnel.funnelSwitchID);

    SmartDashboard.putNumber("Funnel P", 0);
    SmartDashboard.putNumber("Funnel I", 0);
    SmartDashboard.putNumber("Funnel D", 0);
  }

  public void wind() {
    desiredSpeed = Constants.Funnel.winchSpeed;
  }

  public void unwind() {
    desiredSpeed = -Constants.Funnel.winchSpeed;
  }

  public void stop() {
    desiredSpeed = 0;
  }

  public void hold() {
    desiredSpeed = Constants.Funnel.winchHoldSpeed;
  }

  public boolean limitHit() {
    return !funnelLimit.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (limitHit() && desiredSpeed < 0) {
      desiredSpeed = 0;
    }
    motorWinch.set(desiredSpeed);

    SmartDashboard.putNumber("Funnel-Pos", encoderWinch.getPosition());
    SmartDashboard.putBoolean("funnelLimit", limitHit());

    double p = SmartDashboard.getNumber("Funnel P", 0);
    double i = SmartDashboard.getNumber("Funnel I", 0);
    double d = SmartDashboard.getNumber("Funnel D", 0);

    pid.setP(p);
    pid.setI(i);
    pid.setD(d);
  }

  // public void simulationPeriodic() {
  //   double velo = motorWinch.get() * 1.0;
  //   double voltage = RoboRioSim.getVInVoltage();
  //   motorWinchSim.iterate(velo, voltage, Constants.simCycleTime);
  // }
}
