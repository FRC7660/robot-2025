package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Swerve Drive class representing and controlling the swerve drive.
 */
public class SwerveDrive7660 extends SwerveDrive
{
  public SwerveDrive7660(
      SwerveDriveConfiguration config, SwerveControllerConfiguration controllerConfig, double maxSpeedMPS,
      Pose2d startingPose)
  {
      super(config, controllerConfig, maxSpeedMPS, startingPose);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw as a {@link Rotation2d} angle
   */
  public Rotation2d getYaw()
  {
    // Simulation is not broken, only return the negated
    double negation = SwerveDriveTelemetry.isSimulation ? 1.0 : -1.0;
    return Rotation2d.fromRadians(negation * imuReadingCache.getValue().getZ());
  }
}
