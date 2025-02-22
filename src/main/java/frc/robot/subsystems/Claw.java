// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private SparkFlex motorClaw = new SparkFlex(54, MotorType.kBrushless);

  private TalonFXSimState motorClawSim;

  private DigitalInput clawBreakBeam;

  public Claw() {

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(true);
    motorClaw.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    clawBreakBeam =
        new DigitalInput(
            Constants.Claw.clawBeam); // false = beam broken(coral present), true = beam not broken

    // if (Constants.currentMode == Constants.Mode.SIM) {
    //   motorClawSim = new TalonFXSimState(motorClaw);
    // }
  }

  public void start() {
    motorClaw.set(Constants.Claw.clawSpeed);
  }

  public void stop() {
    motorClaw.set(0);
  }

  public boolean getClawSensorHit() {
    return !clawBreakBeam.get(); // false = coral not present, true = coral present
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Claw Break-Beam", getClawSensorHit());
    SmartDashboard.putNumber("Claw Velo", motorClaw.get());
  }

  public void simulationPeriodic() {
    motorClawSim.setSupplyVoltage(
        RobotController
            .getBatteryVoltage()); // need to fix sim capabilites, find talon version of iterate
    // function
  }
}
