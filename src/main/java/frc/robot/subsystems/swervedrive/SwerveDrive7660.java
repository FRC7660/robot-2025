package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.telemetry.SwerveDriveTelemetry;

/** Swerve Drive class representing and controlling the swerve drive. */
public class SwerveDrive7660 extends SwerveDrive {

  private final Lock odometryLock = new ReentrantLock();
  private final SwerveDriveKinematics kinematics;

  public SwerveDrive7660(
      SwerveDriveConfiguration config,
      SwerveControllerConfiguration controllerConfig,
      double maxSpeedMPS,
      Pose2d startingPose) {
    super(config, controllerConfig, maxSpeedMPS, startingPose);
    kinematics = new SwerveDriveKinematics(config.moduleLocationsMeters);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw as a {@link Rotation2d} angle
   */
  public Rotation2d getYaw() {
    // Simulation is not broken, only return the negated
    double negation = SwerveDriveTelemetry.isSimulation ? 1.0 : -1.0;
    return Rotation2d.fromRadians(negation * imuReadingCache.getValue().getZ());
  }

  public Rotation2d getRawYaw() {
    return Rotation2d.fromRadians(imuReadingCache.getValue().getZ());
  }

  public void resetOdometry(Pose2d pose){
    System.out.println("Resetting Odometry");
    odometryLock.lock();
    swerveDrivePoseEstimator.resetPosition(getRawYaw(), getModulePositions(), pose.rotateBy(new Rotation2d(Math.PI)));
    odometryLock.unlock();
  }
}
