// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

  private final XboxController xboxController = Constants.OperatorConstants.xboxController;// This should work - reference to the singular xbox controller in constants. 
  private final SwerveSubsystem swerveSubsystem;
  /*
  * One of the coolest things: Slew Rate Limiter for our joysticks: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html  
  * Limits the rate of change of an output to a maximium rate of change. 
  */ 
  private final SlewRateLimiter
    xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond),
    yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond),
    turningLimiter = new SlewRateLimiter(DriveConstants.kTeleTurningMaxAccelerationUnitsPerSecond);

  
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    // Have the subsystem passed in be a requirement - all other commands that use the command will overide this command. 
    addRequirements(swerveSubsystem); 
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Lets get the robot to move:
    double xSpeed = xboxController.getLeftX();
    double ySpeed = xboxController.getLeftY();
    double turningSpeed = xboxController.getRightX(); // The turning function from right x-axis of xBox.

    // Make sure that x,y, and turning speeds are greater than the deadband - if not, set the speed to zero. 
    xSpeed = Math.abs(xSpeed) > OperatorConstants.kXBoxControllerDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OperatorConstants.kXBoxControllerDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OperatorConstants.kXBoxControllerDeadband ? turningSpeed : 0.0;
  
    // Limit the acceleration given by the controller to ensure that there is a smooth change. 
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxTurningSpeedRadPerSecond;
  
    // Now lets convert that to a chassis speed - which should then be translated to drive and turn speeds required to the modules. 
    ChassisSpeeds chassisSpeeds;
    // Run chassis speeds based on field. 
    // Commented out: NOTE: from field relative means positive y => left of field, positive x => away from driver station. 
    // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, xSpeed, turningSpeed, swerveSubsystem.getRotation2d());
    
    // Relative to robot.
    // X is forward velocity.
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

    // Convert the chassis speeds to the indvidual module states - starting from modules 0 to 3. 
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Apply that to each module - starting in order of FL, FR, BL, BR. 
    swerveSubsystem.setModuleStates(moduleStates);
  }



  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModeules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
