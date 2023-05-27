// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }

  public static class DriveConstants{
    public static final double kPhysicalMaxSpeedMetersPerSecond = 11.5; // According to https://www.andymark.com/products/swerve-and-steer
    
    // We need to update this
    public static final int frontLeftSparkPort = 0;
    public static final int frontLeftTalonId = 0;
    public static final boolean frontLeftDriveReversed = true; // Will I need this?
    public static final boolean frontLeftTurnReversed = true;
  
    public static final int frontRightSparkPort = 1;
    public static final int frontRightTalonId = 1;
    public static final boolean frontRightDriveReversed = false;
    public static final boolean frontRightTurnReversed = true;
  
    public static final int backLeftSparkPort = 2;
    public static final int backLeftTalonId = 2;
    public static final boolean backLeftDriveReversed = true;
    public static final boolean backLeftTurnReversed = true;
  
    public static final int backRightSparkPort = 3;
    public static final int backRightTalonId = 3;
    public static final boolean backRightDriveReversed = false;
    public static final boolean backRightTurnReversed = true;
  
  }
}
