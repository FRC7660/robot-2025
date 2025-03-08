package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import swervelib.parser.SwerveParser;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.json.ModuleJson;

/**
 * Helper class used to parse the JSON directory with specified configuration options.
 */
public class SwerveParser7660 extends SwerveParser {
  public SwerveParser7660(File directory) throws IOException
  {
      super(directory);
  }
  /**
   * Create {@link SwerveDrive} from JSON configuration directory.
   *
   * @param maxSpeed    Maximum speed of the robot in meters per second for normal+angular acceleration in
   *                    {@link swervelib.SwerveController} of the robot
   * @param initialPose {@link Pose2d} initial pose.
   * @return {@link SwerveDrive} instance.
   */
  public SwerveDrive7660 createSwerveDrive(double maxSpeed, Pose2d initialPose)
  {
    SwerveModuleConfiguration[] moduleConfigurations =
        new SwerveModuleConfiguration[moduleJsons.length];
    for (int i = 0; i < moduleConfigurations.length; i++)
    {
      ModuleJson module = moduleJsons[i];
      moduleConfigurations[i] =
          module.createModuleConfiguration(
              pidfPropertiesJson.angle,
              pidfPropertiesJson.drive,
              physicalPropertiesJson.createPhysicalProperties(),
              swerveDriveJson.modules[i]);
    }
    SwerveDriveConfiguration swerveDriveConfiguration =
        new SwerveDriveConfiguration(
            moduleConfigurations,
            swerveDriveJson.imu.createIMU(),
            swerveDriveJson.invertedIMU,
            physicalPropertiesJson.createPhysicalProperties());

    return new SwerveDrive7660(
        swerveDriveConfiguration,
        controllerPropertiesJson.createControllerConfiguration(swerveDriveConfiguration, maxSpeed),
        maxSpeed,
        initialPose);
  }
}

