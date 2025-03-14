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

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.math.Matter;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double simCycleTime = 0.05;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Arm {
    public static double armSpeed = 0.1;
    public static double armEncoderUpperLimit = 1;
    public static double armEncoderLowerLimit = 4;
    public static final int motorID = 53;
    public static final double kDt = 0.02;
    public static final double kMaxVelocity = 50;
    public static final double kMaxAcceleration = 50;
    public static final double kS = 0.0;
    public static final double kGVoltage = 0.28; // multiplied by 12 for voltage
    public static final double kV = 0.125;
    public static final double kA = 0.0;
    public static final double kp = 1.5;
    public static final double ki = 0.025;
    public static final double kd = 0;
    public static final double scorePos = -2.8;
    public static final double zeroPos = 0.0;
    public static final double safePosIn = -2.45;
    public static final double safePosOut = -10.0;
    public static final double reverseLimit = -450;
    public static final double forewardLimit = -14;
    public static final double testPosition = -380;
    public static final double horizontalCounts = -12.86;
    public static final double countsPerRadian = 5.83;
    public static final double gravityFeedForward = 0.07;
    public static final double motorOffset = -12.3;
    public static final double motorRotationsPerCircle = 37.2;
    public static final double radiansPerMotorRotation = (2 * Math.PI) / 37.2;
    public static final double endDeadband = 0.4;

    public enum Direction {
      OUT,
      IN,
    }
  }

  public static class Claw {
    public static double clawSpeed = 0.2;
    public static final int clawBeam = 4;
    public static final int motorID = 54;
  }

  public static class Climb {
    public static int MotorClimbID = 31;
    public static double climbSpeed = 0.50;
    public static double climbEncoderLimit = 1;
    public static int climbSwitchID = 5;
    public static double upperLimit = 270;
    public static double halfwayPosition = 80;
  }

  public static class Funnel {
    public static int winchID = 41;
    public static double winchSpeed = 0.1; // Clockwise = positive, holds funnel down in position
    public static double limit = 1;
    public static int funnelSwitchID = 6;
  }

  public static class ButtonBox {
    public static final int bottomLeft = 1;
    public static final int lowerLeft = 2;
    public static final int upperLeft = 3;
    public static final int topLeft = 4;
    public static final int bottomRight = 5;
    public static final int lowerRight = 6;
    public static final int upperRight = 7;
    public static final int topRight = 8;
    public static final int p1 = 9;
    public static final int p2 = 10;
  }

  public enum ElevatorState {
    L1,
    L2,
    L3,
    L4,
    ZERO
  }

  public static final Double l1height = 2.0; // 60 * 0.25;
  public static final Double l2height = 10.5; // 60 * 0.50;
  public static final Double l3height = 28.5; // 60 * 0.75;
  public static final Double l4height = 58.5; // 60 * 0.99;
  public static final Double zeroHeight = 0.0;

  public static final Double elevatorP = 0.1;
  public static final Double elevatorI = 0.0;
  public static final Double elevatorD = 0.0;

  public static class Elevator {
    public static final Double lowerLimit = 1.5;
    public static final Double upperLimit = 60.0;
    public static final Double feedForward = 0.4;
    public static final int lowerlimitID = 3;
    public static final int motorAlphaID = 51;
    public static final int motorBetaID = 52;
    public static final double manualOutput = .2 * 12; // output in volts

    public enum Direction {
      UP,
      DOWN,
    }
  }

  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  public static final double ROBOT_MASS = 125 * 0.453592;
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double DEADBAND = 0.1;

  // in meters per second. value is max strafe speed.
  public static final double strafeSpeedMultiplier = 4;

  public static final boolean absoluteDrive = false;
}
