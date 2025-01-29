import net.java.games.input.*;

public class JoystickTest {
    public static void main(String[] args) {
        // Get all available controllers
        ControllerEnvironment environment = ControllerEnvironment.getDefaultEnvironment();
        Controller[] controllers = environment.getControllers();

        // Find a joystick
        Controller joystick = null;
        for (Controller controller : controllers) {
            if (controller.getType() == Controller.Type.STICK || controller.getType() == Controller.Type.GAMEPAD) {
                joystick = controller;
                break;
            }
        }

        if (joystick == null) {
            System.out.println("No joystick detected. Please connect your USB encoder or joystick.");
            return;
        }

        System.out.println("Joystick connected: " + joystick.getName());

        // Event loop
        while (true) {
            joystick.poll(); // Poll the joystick for updates
            EventQueue queue = joystick.getEventQueue();
            Event event = new Event();

            while (queue.getNextEvent(event)) {
                Component component = event.getComponent();
                float value = event.getValue();

                if (component.isAnalog()) {
                    System.out.println("Axis " + component.getName() + " moved to " + value);
                } else {
                    if (value == 1.0f) {
                        System.out.println("Button " + component.getName() + " pressed!");
                    } else {
                        System.out.println("Button " + component.getName() + " released!");
                    }
                }
            }

            try {
                Thread.sleep(50); // Sleep to reduce CPU usage
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
