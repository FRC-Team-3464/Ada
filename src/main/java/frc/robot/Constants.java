// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Constants related to individual swerve modules. 
  public static class ModuleConstants{
    public static final double kPTurning = 0.255; 
    public static final double kTurnEncoderRatio = 1/1656.66667; // 7 * 71 * (40/48) * 4;
    public static final double kTurningEncoderRot2Rad = kTurnEncoderRatio * (2 * Math.PI); // Verified
    public static final double kTurningEncoderUnitP100ms2RadPerSec = kTurningEncoderRot2Rad * 10; // Talon SRX reads velocity in units/100ms. https://v5.docs.ctr-electronics.com/en/stable/ch14_MCSensor.html 
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 3;

    public static final double kXBoxControllerDeadband = 0.05; // Will need to be updated later - only 5%?
    public static final XboxController xboxController = new XboxController(kDriverControllerPort); // This controller should be used for all commands - static mans only one copy allowed. 
  }

  public static class DriveConstants{

    // Left and right wheel distance. 
    public static final double kTrackWidth = Units.inchesToMeters(22);
    // Front and Back Wheel distance. 
    public static final double kWheelBase = Units.inchesToMeters(24); // More long than wide. 
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase/2, -kTrackWidth/2), // Positition of mod 0 - front left
      new Translation2d(kWheelBase/2, kTrackWidth/2), // Positition of mod 1 - front right
      new Translation2d(-kWheelBase/2, -kTrackWidth/2), // Positition of mod 2 - back left
      new Translation2d(-kWheelBase/2, kTrackWidth/2) // Positition of mod 3 - back right. 
    );

    // Actual Physical Robot Constraints:
    // Drive Meters 11.5 ft to meters: ~3.5 meters per second.
    public static final double kPhysicalMaxDriveSpeedMetersPerSecond = Units.feetToMeters(11.5); // According to https://www.andymark.com/products/swerve-and-steer
    // Max speed: 1.5 Rotations per second (90 RPM) * 2pi radians/one rotation
    public static final double kPhysicalMaxTurningSpeedRadPerSecond = 1.5 * 2 * Math.PI; 

    // Tele-Op Robot Constraints - change... it should be smaller than that. 
    // Dividing by four = splitting up the total speed into each of the modules: used to ensure that our swerve isn't too fast. 
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxDriveSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxTurningSpeedRadPerSecond = kPhysicalMaxTurningSpeedRadPerSecond / 4;

    // Using comparative proportions, we get 3/5 = x/3.5
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2.1;
    public static final double kTeleTurningMaxAccelerationUnitsPerSecond = 2.1;
    


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
