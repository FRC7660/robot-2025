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

import edu.wpi.first.wpilibj.RobotBase;

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
    public static double armSpeed = 0.50;
    public static double armEncoderUpperLimit = 1;
    public static double armEncoderLowerLimit = 4;
    public static final int motorID = 53;
    public static final double safe_pos = 0.4;
    public static final double kp = 0.1;
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double reverseLimit = -450;
    public static final double forewardLimit = -10;
    public static final double testPosition = -380;
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
    public static double upperLimit = 225;
  }

  public static class Funnel {
    public static int winchID = 41;
    public static double winchSpeed = 0.05; // Clockwise = positive, holds funnel down in position
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

  public static final Double l1height = 25.0;
  public static final Double l2height = 50.0;
  public static final Double l3height = 75.0;
  public static final Double l4height = 100.0;
  public static final Double zeroHeight = 0.0;

  public static final Double elevatorP = 0.1;
  public static final Double elevatorI = 0.0;
  public static final Double elevatorD = 0.0;

  public static class Elevator {
    public static final Double lowerLimit = 0.0;
    public static final Double upperLimit = 100.0;
    public static final int lowerlimitID = 3;
    public static final int motorAplphaID = 51;
    public static final int motorBetaID = 52;
  }

  public static final boolean absoluteDrive = false;
}
