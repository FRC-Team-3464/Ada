package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final Spark driveMotor; // Change to Spark
    private final TalonSRX turningMotor; // Change to Talon


    private final PIDController turningPidController; // Add a controller for our turning motor. 
    // private final TalonSRXSimCollection simTurningMotor; // Create sim object
    
    // Constructor where we update everything we need to create a module. 
    public SwerveModule(int SparkPort, int TalonId, boolean driveMotorReversed, boolean turningMotorReversed){
        // Define motors to be those we created with passed in ports. 
        driveMotor = new Spark(SparkPort);
        turningMotor = new TalonSRX(TalonId);

        // Set motors to be inversed if we told it to. 
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        // simTurningMotor = turningMotor.getSimCollection(); // Add new sim object
    
        turningPidController = new PIDController(0.1, 0, 0); // Double check value. 
        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // Basically controller moves to find the shortest path to a target in a circle - a circle's diameter is 2pi. 
        
        // Will need to make that constant but can't use for the encoder. 
        // turningMotor.getSelectedSenor.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        resetEncoders(); // Call our method to reset our encoder when we create the module. 
    };

    public double getTurningPosition(){
        // Return Talon Turning Position
        // Gear RATIO IS WRONG
        return (turningMotor.getSelectedSensorPosition(0) * ModuleConstants.kTurningEncoderRot2Rad); 
    }

    public double getTurningVelocity(){
        // Return Talon Turning Position
        // Gear RATIO IS WRONG
        // pidId 0 is simply the quadratic encoder - can be set with the phoenix tuner. 

        return (turningMotor.getSelectedSensorVelocity(0) * ModuleConstants.kTurningEncoderUnitP100ms2RadPerSec);    
    }

    public void resetEncoders() {
        // NOTE: The ms means that the code will throw an error after desired ms - at 100 ms in this example. 
        turningMotor.setSelectedSensorPosition(0, 0,100);  // pidId 0 is simply the quadratic encoder - can be set with the phoenix tuner. 
    }

    // Create a rotation 2d that symbolizes the 2d rotation state of the wheel. 
    public Rotation2d rotationState(){
        // We nmay need a drive encoder
        return new Rotation2d(getTurningPosition());
    }

    public void setDesiredState(SwerveModuleState state){
        //  Make sure that we're actually wanting to change the speed - if we're just letting go of the controller, we don't need to get to zero
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            // If the speed is too insignificant - don't bother
            stopMotors();
            return; // Exit function
        }
        state = SwerveModuleState.optimize(state, rotationState()); // Have the passed in state get translated so we now just need the shortest possible path for the wheel to rotate. 
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxDriveSpeedMetersPerSecond); // Give us a percentage speed for the spark to go to.  - Make sure it doesn't cross 1!
        turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));        
        // Shuffleboard.getTab("Smar")
        
        // Simulation Support
        // simTurningMotor.setBusVoltage(getTurningPosition());

        SmartDashboard.putString("Swerve[" + driveMotor.getChannel() + "] state:", state.toString()); // Give us the module debug info. .
        SmartDashboard.putNumber("Swerve[" + driveMotor.getChannel() + "] turn", state.angle.getDegrees()); 
        SmartDashboard.putData("Swerve[" + driveMotor.getChannel() + "] PID Controller ", turningPidController); 

        SmartDashboard.putNumber("Swerve[" + driveMotor.getChannel() + "] Speed ", state.speedMetersPerSecond / DriveConstants.kPhysicalMaxDriveSpeedMetersPerSecond); 
        
    }

    public void stopMotors(){
        // driveMotor.set(0);
        driveMotor.stopMotor();
        turningMotor.set(ControlMode.PercentOutput, 0);
    }


}
