// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private final CANBus kCANBus = new CANBus("canivore");

  private TalonFX motorArm = new TalonFX(81, kCANBus); 

  private TalonFXSimState motorArmSim;

  public Arm() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      motorArmSim = new TalonFXSimState(motorArm);
    }

    motorArm.setPosition(0);

    motorArm.setNeutralMode(NeutralModeValue.Brake);
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
    SmartDashboard.putNumber("Arm-Pos", motorArm.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm-Velo", motorArm.getVelocity().getValueAsDouble());
  }

   public void simulationPeriodic() {
    motorArmSim.setSupplyVoltage(RobotController.getBatteryVoltage());
  }
}
