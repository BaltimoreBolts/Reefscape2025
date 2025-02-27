// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
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
  }
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class ElevatorConstants 
  { // set values
    public static final int kElevatorLeft = 10;
    public static final boolean kMotorInverted1 = false;
    public static final int kElevatorRight = 11;
    public static final boolean kMotorInverted2 = true;
    public static final int kMinPosition = 0;
    public static final double kP = 6.8; //more p 
    public static final double kI = 0.0;//003;
    public static final double kIz = 0;
    public static final double kD = 6.0; //change as needed
    public static final double kFF = 0.07;
    public static final double kSpeedLimitFactor = 0.5;
    public static final double kToleranceRotations = 0.2;
    public static final double kArmSpeedModifier = 0.25;

    public enum ElevatorState {
      ZERO(0.0),
      RESTING(1.0),
      SCORE_L2(7.5),
      SCORE_L3(10.8),
      SCORE_L4(17.2);

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

    public static final int kShooterRightPort = 12;
    public static final int kShooterLeftPort = 13;

    // set speed?
    public static final double kShooterSpeed = 1.0; 

    public enum ScoringTarget {
      L2,
      L3,
      L4
    }
  }

}
