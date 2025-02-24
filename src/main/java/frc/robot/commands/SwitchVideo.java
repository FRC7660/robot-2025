package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwitchVideo extends Command{

    private final CommandXboxController controller;
    private final Trigger bButton;

    public SwitchVideo(CommandXboxController controller) {

        this.controller = controller;
        this.bButton = controller.b();
    }

    @Override
    public void execute() {
        SmartDashboard.getEntry("cam_num");
        SmartDashboard.putNumber("cam_num", 2);
    }
}