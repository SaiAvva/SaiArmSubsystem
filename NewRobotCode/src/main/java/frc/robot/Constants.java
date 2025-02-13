// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kJoystickPort = 1;
  }

  public static class SwerveConstants{
    public static double maximumSpeed = Units.feetToMeters(4.5);
    public static double deadband = 0.05;
  }

  public static class ArmConstants{
  //Id Ports for Motors
  public static final int baseMotorIDPort = 0;
  public static final int armMotorIDPort = 1;
  public static final int endEffectorMotorIDPort = 3;
  public static final int intakeMotorIDPort = 4;


  //PID Constants for the base motor
  public static final double kP_BASE = 0.1;
  public static final double kI_BASE = 0.0;
  public static final double kD_BASE = 0.0;
  private static final double kFF_BASE = 0.0;
  private static final double MAX_Output = 1.0;
  private static final double MIN_Output = -1.0;

  //Gear Ratios for Arm Motors
  public final static double baseGearRatio = 1/51f;
  public final static double armGearRatio = 25/1f;
  public final static double endEffectorGearRatio = 1;
  public final static double intakeMotorGearRatio = 1;

  //Preset Positions to move the arm (Base Motor)
  public static final double POSITION_LOW = 10.0;
  public static final double POSITION_MID = 30.0;
  public static final double POSITION_HIGH = 60.0;
  }

  //Setpoint Limits
  public static final double POSITION_MAX = 100.0;
  public static final double POSITION_MIN = 0.0;
}
