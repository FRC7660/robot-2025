// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.commands.ManualElevator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


//import com.revrobotics.CANSparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {

  public final SparkMax motorAlpha = new SparkMax(46, MotorType.kBrushless);
  //public final SparkMax motorBeta = new SparkMax(46, MotorType.kBrushless);
  public int alphaInversion = 1; // The factor by which a motor's rotation should be applied
  public int betaInversion = 1;
  public DigitalInput bottomLimit = new DigitalInput(1);
  public DigitalInput topLimit = new DigitalInput(2);
  public SparkMaxConfig alphaConfig = new SparkMaxConfig();
  public SparkMaxConfig betaConfig = new SparkMaxConfig();
  
  private RelativeEncoder motorAlphaEncoder = motorAlpha.getEncoder();
  //private RelativeEncoder motorBetaEncoder = motorBeta.getEncoder();

  //private SparkMaxSim motorSim = new SparkMaxSim(motorAlpha,DCMotor.getNeo550(1));

  public void raise() {
    motorAlpha.set(.5*alphaInversion);
    //motorBeta.set(.5*betaInversion);
  }

  public void lower() {
    motorAlpha.set(-.5*alphaInversion);
    //motorBeta.set(-.5*betaInversion);
  }

  public void setMotor(double speed) {
    Double outputSpeed = MathUtil.applyDeadband(speed, 0.1);
    motorAlpha.set(outputSpeed*alphaInversion);
    //motorBeta.set(outputSpeed*betaInversion);
    //motorSim.setAppliedOutput(outputSpeed);
  }

  public Double getHeight(){
    Double height = 99.9;
    Double baseRotations = motorAlphaEncoder.getPosition();
    Double translatedRotations = baseRotations/6;
    return height;
  }

  public void setL1(){

  }

  public Command runManualCommand(DoubleSupplier inputSpeed){
    
    return this.runOnce(() -> setMotor(inputSpeed.getAsDouble()));
  }

  public Elevator() {
    // Clockwise - up, Counter - down, same for both motors
    // Gear ratio - 6:1
    //motorSim.setPosition(5);
    
    alphaConfig.softLimit.forwardSoftLimit(100);
    alphaConfig.softLimit.reverseSoftLimit(0);
    alphaConfig.softLimit.forwardSoftLimitEnabled(true);
    alphaConfig.softLimit.reverseSoftLimitEnabled(true);
    alphaConfig.idleMode(IdleMode.kBrake);
    betaConfig = alphaConfig;
    
    motorAlpha.configure(alphaConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
    //motorBeta.configure(betaConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Motor Sim Speed", motorSim.getAppliedOutput());
    //SmartDashboard.putNumber("Motor Sim Position", motorSim.getAbsoluteEncoderSim().getPosition());
  }
}
