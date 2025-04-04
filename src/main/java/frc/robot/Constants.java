// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class inputConstants {
        public static final class digitalInputConstants {
            public static final int kFirstSensor = 8;
            public static final int kSecondSensor = 9;
        }
    }

    public static class ControllerConstants {
        public static final double kDeadzone = .1;
        public static final double kTriggerDeadzone = .05;

        public static final class Axis {
            public static final int kLeftX = 0;
            public static final int kLeftY = 1;
            public static final int kRightX = 4;
            public static final int kLeftTrigger = 2;
            public static final int kRightTrigger = 3;
            public static final int kRightY = 5;
        }

        public static final class Button {
            public static final int kA = 1;
            public static final int kB = 2;
            public static final int kX = 3;
            public static final int kY = 4;
            public static final int kLeftBumper = 5;
            public static final int kRightBumper = 6;
            public static final int kLeftMenu = 7;
            public static final int kRightMenu = 8;
            public static final int kLeftTriggerButton = 9;
            public static final int kRightTriggerButton = 10;
        }

        public static final class DPad {
            public static final int kUp = 0;
            public static final int kRight = 90;
            public static final int kDown = 180;
            public static final int kLeft = 270;
        }
    }

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS =
            new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    // Maximum speed of the robot in meters per second, used to limit acceleration.

    //  public static final class AutonConstants
    //  {
    //
    //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
    //  }
    public static final double X_REEF_ALIGNMENT_P = 3.3;
    public static final double Y_REEF_ALIGNMENT_P = 3.3;
    public static final double ROT_REEF_ALIGNMENT_P = 0.058;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT = -12.63; // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.09; // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.54; // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;

    public static final class DrivebaseConstants {

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants {

        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static final class ElevatorConstants { // set values
        public static final int kElevatorLeft = 10;
        public static final boolean kMotorInverted1 = false;
        public static final int kElevatorRight = 11;
        public static final boolean kMotorInverted2 = true;
        public static final int kMinPosition = 0;
        public static final double kSpeedLimitFactor = 0.25;
        public static final double kToleranceRotations = 0.2;
        public static final double kArmSpeedModifier = 0.25;

        // Trapezoidal, FF, PID
        public static final double kMaxV = 40.0;
        public static final double kMaxA = 35.0;
        public static final double kS = 0.0;
        public static final double kG = 0.25;
        public static final double kV = 0.25;
        public static final double kP = 0.09;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public enum ElevatorState { // 2 enc values per inch
            ZERO(0.0),
            RESTING(1.0),
            // TODO: maybe add new constant or rename to be more general
            INTAKE_ALGAE(4.0), // lazy add to elevator
            STOW_ALGAE(0.0), // lazy add to elevator
            SCORE_L1(20.0), // 10inch travel; put bottom edge of coral at 20in off ground
            SCORE_L2(29.0), // 20inch travel; put center of coral at 32in off ground
            SCORE_L3(49.0),
            SCORE_L4(82.0),
            BARGE(85);

            final double position;

            ElevatorState(double position) {
                this.position = position;
            }

            public double getPosition() {
                return position;
            }
        }
    }

    public static final class ShooterConstants {
        // set ports

        public static final int kShooterPort = 13;

        // set speed
        public static final double kShooterSpeed = -0.6;
        public static final double kIntakeSpeed = -0.4;
        public static final double kOutSpeed = 0.4;

        public enum ScoringTarget {
            L1,
            L2,
            L3,
            L4,
            BARGE
        }
    }

    public static final class AlgaeConstants {
        // set ports

        public static final int kAlgaeRollerPort = 12; // currently as intake
        public static final int kAlgaePivotPort = 14; // currently as pivot

        // set speed
        public static final double kIntakeSpeed = 0.65;
        public static final double kOuttakeSpeed = -0.65;

        public enum AlgaeScoringTarget {
            INTAKE_ALGAE,
            STOW_ALGAE
        }
    }
}
