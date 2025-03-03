// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private final double FIXME_DOUBLE = -1.11111111;

  public Drive() {}

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return FIXME_DOUBLE; // maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return FIXME_DOUBLE; // maxSpeedMetersPerSec / driveBaseRadius;
  }

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    // FIXME
    return;
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    return new Pose2d();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return new Rotation2d();
  }

  public void resetGyro() {
    // FIXME
    return;
  }
}
