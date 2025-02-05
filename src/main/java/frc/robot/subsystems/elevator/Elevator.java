// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorState;

import java.util.function.DoubleSupplier;

//import com.revrobotics.CANSparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {

  public final SparkMax motorAlpha = new SparkMax(3, MotorType.kBrushless);
  public final SparkMax motorBeta = new SparkMax(0, MotorType.kBrushless);
  public int alphaInversion = -1; // The factor by which a motor's rotation should be applied
  public int betaInversion = -1;
  public DigitalInput bottomLimit = new DigitalInput(1);
  public DigitalInput topLimit = new DigitalInput(2);
  public SparkMaxConfig alphaConfig = new SparkMaxConfig();
  public SparkMaxConfig betaConfig = new SparkMaxConfig();

  private RelativeEncoder motorAlphaEncoder = motorAlpha.getEncoder();
  private RelativeEncoder motorBetaEncoder = motorBeta.getEncoder();

  private SparkMaxSim motorSim = new SparkMaxSim(motorAlpha,DCMotor.getNeo550(1));
  private SparkRelativeEncoderSim motorSimEncoder = motorSim.getRelativeEncoderSim();

  private PIDController elevatorPid = new PIDController(
    Constants.elevatorP,Constants.elevatorI,Constants.elevatorD);

  public void raise() {
    motorAlpha.set(.5 * alphaInversion);
    motorBeta.set(.5*betaInversion);
  }

  public void lower() {
    motorAlpha.set(-.5 * alphaInversion);
    motorBeta.set(-.5*betaInversion);
  }

  public void setMotors(double speed,boolean overrideInversion) {
    Double speedMulti = 0.2;
    Double outputSpeed = speed;
    if (overrideInversion == true){
      motorAlpha.set(outputSpeed * speedMulti);
      motorBeta.set(outputSpeed * speedMulti);
    } else {
      motorAlpha.set(outputSpeed * speedMulti * alphaInversion);
      motorBeta.set(outputSpeed * speedMulti *betaInversion);
    }
    SmartDashboard.putNumber("Motor Alpha OutputSpeed", outputSpeed);
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
    switch(state){
      case L1:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(),Constants.l1height);
        break;
      case L2:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(),Constants.l2height);
        break;
      case L3:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(),Constants.l3height);
        break;
      case L4:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(),Constants.l4height);
        break;
      case ZERO:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(),Constants.zeroHeight);
        break;
      
      default:
        output = elevatorPid.calculate(motorAlphaEncoder.getPosition(),Constants.l1height);
        break;      
    }
    setMotors(output*0.2,true);

  }

  public Command runManualCommand(DoubleSupplier inputSpeed) {

    return this.runOnce(() -> setMotors(inputSpeed.getAsDouble(),false));
  }

  public Elevator() {
    // Clockwise - up, Counter - down, same for both motors
    // Gear ratio - 6:1
    //motorSim.setPosition(5);
    

    motorAlphaEncoder.setPosition(50);
    System.out.println("Motor Position:"+motorAlphaEncoder.getPosition());

    alphaConfig.softLimit.forwardSoftLimit(100);
    alphaConfig.softLimit.reverseSoftLimit(0);
    alphaConfig.softLimit.forwardSoftLimitEnabled(true);
    alphaConfig.softLimit.reverseSoftLimitEnabled(true);
    alphaConfig.idleMode(IdleMode.kBrake);
    betaConfig = alphaConfig;

    motorAlpha.configure(
        alphaConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    motorBeta.configure(betaConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Motor Alpha Speed", motorAlpha.get());
    SmartDashboard.putNumber("Motor Alpha Position", motorAlphaEncoder.getPosition());
    // SmartDashboard.putNumber("Motor Sim Position",
    //   motorSim.getRelativeEncoderSim().getPosition());
  }
}
