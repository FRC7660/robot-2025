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

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {

    public static boolean talonDriveMotors = true;

    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(26.5);
    public static final double wheelBase = Units.inchesToMeters(26.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(4.864253078385368); //FIXME IN RADIANS
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(5.203262832508095); //FIXME IN RADIANS
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(3.926990816987241); //FIXME IN RADIANS
    public static final Rotation2d backRightZeroRotation = new Rotation2d(2.030990563160589); //FIXME IN RADIANS

    // Device CAN IDs
    public static final int pigeonCanId = 9;

    public static final int frontLeftDriveCanId = 10;
    public static final int backLeftDriveCanId = 15;
    public static final int frontRightDriveCanId = 20;
    public static final int backRightDriveCanId = 25;

    public static final int frontLeftTurnCanId = 11;
    public static final int backLeftTurnCanId = 16;
    public static final int frontRightTurnCanId = 21;
    public static final int backRightTurnCanId = 26;

    public static final int frontLeftEncoderID = 12;
    public static final int backLeftEncoderID = 17;
    public static final int frontRightEncoderID = 22;
    public static final int backRightEncoderID = 27;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 60;
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double driveMotorReduction =
            6.12;
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
            2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
            (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    public static final double slipCurrent = 120.0;
    public static final ClosedLoopOutputType driveMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // Drive PID configuration
    public static final double driveKp = 0.0; //FIXME
    public static final double driveKd = 0.0; //FIXME
    public static final double driveKs = 0.0; //FIXME
    public static final double driveKv = 0.1; 
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 150.0 / 7.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1); //FIXME

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 0.7; // Adjust this value as needed
    public static final double turnKi = 0; // Add this value and adjust as needed
    public static final double turnKd = 0.01;  // Adjust this value as needed
    public static final double turnSimP = 8.0; 
    public static final double turnSimD = 0.0; 
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 45; //FIXME
    public static final double robotMOI = 6.883; //FIXME
    public static final double wheelCOF = 1.2; //FIXME
    public static final RobotConfig ppConfig = new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                    wheelRadiusMeters,
                    maxSpeedMetersPerSec,
                    wheelCOF,
                    driveGearbox.withReduction(driveMotorReduction),
                    driveMotorCurrentLimit,
                    1),
            moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(moduleTranslations)
            .withRobotMass(Kilogram.of(robotMassKg))
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    driveGearbox,
                    turnGearbox,
                    driveMotorReduction,
                    turnMotorReduction,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    Meters.of(wheelRadiusMeters),
                    KilogramSquareMeters.of(0.02),
                    wheelCOF));
}
