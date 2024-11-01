// Copyright 2021-2024 FRC 6328
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

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Ports {
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontLeftTurningCanId = 1;

    public static final int kFrontRightDrivingCanId = 6;
    public static final int kFrontRightTurningCanId = 5;

    public static final int kRearLeftDrivingCanId = 4;
    public static final int kRearLeftTurningCanId = 3;

    public static final int kRearRightDrivingCanId = 8;
    public static final int kRearRightTurningCanId = 7;

    public static final int kGyroId = 9;

    public static final int kClimberLeaderID = 11;
    public static final int kClimberFollowerID = 12;

    public static final int kIndexerMotorId = 15;
    public static final int kTopFlywheelMotorId = 16;
    public static final int kBottomFlywheelMotorId = 17;
    public static final int kShieldMotorId = 18;
    public static final int kPivotMotorLeaderId = 13;
    public static final int kPivotMotorFollowerId = 14;

    public static final int kIntakeMotorID = 10;
  }

  public static final class PIDConstants {
    public static final double kClimberP = 0.5;
    public static final double kClimberI = 0;
    public static final double kClimberD = 0;
    
  }

  public static final class ElevatorConstants {
    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;

    public static final double kElevatorKp = 5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;

    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.762; // volts (V)
    public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kElevatorGearing = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass = 4.0; // kg

    public static final double kSetpointMeters = 0.75;
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 1.25;

    // distance per pulse = (distance per revolution) / (pulses per revolution)
    //  = (Pi * D) / ppr
    public static final double kElevatorEncoderDistPerPulse =
        2.0 * Math.PI * kElevatorDrumRadius / 4096;
  }
}
