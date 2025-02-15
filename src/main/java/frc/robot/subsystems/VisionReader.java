// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionReader extends SubsystemBase {
  /** Creates a new VisionReader. */
  public VisionReader() {
    // ObjectMapper objectMapper = new ObjectMapper();

    // String jsonString = jsonEntry.getString("{}");

    // @SuppressWarnings("unchecked")
    // Map<String, Object> map = objectMapper.readValue(jsonString, Map.class);
    // System.out.println(map); // Output: {name=Alice, age=28}
    // } catch (Exception e) {
    // e.printStackTrace();
    // }
  }

  @Override
  public void periodic() {
    String dataThing = SmartDashboard.getString("detecTable", "{}");
    System.out.println(dataThing);
    // This method will be called once per scheduler run
  }
}
