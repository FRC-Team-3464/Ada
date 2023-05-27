// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.frontLeftSparkPort,
    DriveConstants.frontLeftTalonId,
    DriveConstants.frontLeftDriveReversed,
    DriveConstants.frontLeftTurnReversed
  );

  public final SwerveModule frontRight = new SwerveModule(
    DriveConstants.frontRightSparkPort,
    DriveConstants.frontRightTalonId,
    DriveConstants.frontRightDriveReversed,
    DriveConstants.frontRightTurnReversed
  );

  public final SwerveModule backLeft = new SwerveModule(
    DriveConstants.backLeftSparkPort,
    DriveConstants.backLeftTalonId,
    DriveConstants.backLeftDriveReversed,
    DriveConstants.backLeftTurnReversed
  );

  public final SwerveModule backRight = new SwerveModule(
    DriveConstants.backRightSparkPort,
    DriveConstants.backRightTalonId,
    DriveConstants.backRightDriveReversed,
    DriveConstants.backRightTurnReversed
  );
    
  private AHRS gyro = new AHRS(Port.kMXP); // Import our gyroscope

  public SwerveSubsystem() {
    new Thread(() -> { // Create two parallel threads: one runs everything else while another one resets the gyro after one second. 
      try{
        Thread.sleep(1000, 0);
        zeroHeading();
      }catch (Exception e){
    }}).start();
  }

  public void zeroHeading(){
    gyro.reset(); // Reset gyro
  }

  public double getHeading(){
    // Get our heading to be between -180 and 180. 
    return Math.IEEEremainder(gyro.getAngle(), 360); // How does this work? Supposed to clamp angle to between -180 to 180... I guess 360 is the full range of moation and we start at 0... 
  }

  public Rotation2d getRotation2d(){
    // Need to look into what continuous and not continous means. 
    return Rotation2d.fromDegrees(getHeading()); // return a new rotation 2D based on the gyro heading. 
  }

  public void stopModeules(){
    // Stop all four modules. 
    frontLeft.stopMotors();
    frontRight.stopMotors();
    backLeft.stopMotors();
    backRight.stopMotors(); 
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    /*
    * Make sure that if, we ask for a maximium speed from one of the modules, the others are proportianally lower - compared to having others being proportinally larger and thus exceed their limits too. 
    * Note that desaturateWheelSpeeds is the code replacement for normalize wheel speeds.
    */ 
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxDriveSpeedMetersPerSecond);
    // With a given array of swerve module states - forward/backward speeds and rotation requests - to each of the corresponding modules. 
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);

  }

  @Override
  public void periodic() {
    // Send to Smartdashboard our heading constantly. 
    SmartDashboard.getNumber("Robot Gyro Heading", getHeading()); // Could we try getting our rotation2D?
    
  }
}
