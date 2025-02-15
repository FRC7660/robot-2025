// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private final CANBus kCANBus = new CANBus("canivore");

  private TalonFX motorClaw = new TalonFX(71, kCANBus);

  private TalonFXSimState motorClawSim;

  private DigitalInput clawBreakBeam =
      new DigitalInput(
          Constants.Claw.clawBeam); // true = beam broken(coral present), false = beam not broken

  public Claw() {

    motorClaw.setNeutralMode(NeutralModeValue.Brake);

    motorClaw.setPosition(0);

    if (Constants.currentMode == Constants.Mode.SIM) {
      motorClawSim = new TalonFXSimState(motorClaw);
    }
  }

  public void start() {
    motorClaw.set(Constants.Claw.clawSpeed);
  }

  public void stop() {
    motorClaw.set(0);
  }

  public boolean getClawSensorHit() {
    return !clawBreakBeam.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Claw Break-Beam", getClawSensorHit());
    SmartDashboard.putNumber("Claw-Motor", motorClaw.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Claw Velo", motorClaw.get());
  }

  public void simulationPeriodic() {
    motorClawSim.setSupplyVoltage(
        RobotController
            .getBatteryVoltage()); // need to fix sim capabilites, find talon version of iterate
    // function
  }
}
