// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(); // Create an object of the class SwerveSubsystem. 
  
  public RobotContainer() {
    // Set default swerve command. 
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem)); // Keep calling this... And we're done!
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(Constants.OperatorConstants.xboxController, 1).onTrue(() -> swerveSubsystem.zeroHeading());

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
