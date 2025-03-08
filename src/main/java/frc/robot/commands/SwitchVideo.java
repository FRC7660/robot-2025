package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwitchVideo extends Command {

  public SwitchVideo() {}

  @Override
  public void initialize() {

    int foundIndex = -1;
    int nextNum = 0;
    double current = -1;
    double currentNum = SmartDashboard.getNumber("camera_num", current);
    SmartDashboard.putNumber("camera_num", 2);

    double[] normal = null;
    double[] cameras = SmartDashboard.getNumberArray("cameras", normal);
    if (cameras == normal) {
      return;
    }

    for (int i = 0; i < cameras.length; i++) {
      if (cameras[i] == currentNum) {
        foundIndex = i;
        break;
      }
    }

    nextNum = (int) (cameras[(foundIndex + 1) % cameras.length]);

    SmartDashboard.putNumber("camera_num", nextNum);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
