package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
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

        // Refine when testing. 
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0); 

        // Tell the robot that the encoder is circuler -PI to PI. 
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders(); // Call our method to reset our encoder when we create the module. 
    };

    public void resetEncoders() {
        // NOTE: The ms means that the code will throw an error after desired ms - at 100 ms in this example. 
        turningMotor.setSelectedSensorPosition(0, 0,100);  // pidId 0 is simply the quadratic encoder - can be set with the phoenix tuner. 
    }


    public double getTurningPosition(){
        // Return Talon Turning Position in Rads
        return (turningMotor.getSelectedSensorPosition(0) * ModuleConstants.kTurningEncoderRot2Rad); 
    }

    public double getTurningVelocity(){
        // Return Talon Turning Position in Rads/Sec
        // pidId 0 is simply the quadratic encoder - can be set with the phoenix tuner. 
        return (turningMotor.getSelectedSensorVelocity(0) * ModuleConstants.kTurningEncoderUnitP100ms2RadPerSec);    
    }


    // Create a rotation 2d that symbolizes the 2d rotation state of the wheel. 
    public Rotation2d getRotationState(){
        return new Rotation2d(getTurningPosition());
    }

    // Actually move the modules. 
    public void setDesiredState(SwerveModuleState state){
        //  Don't run this if the change is too small. 
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stopMotors();
            return; // Exit function
        }

        // Find the shortest possible path for the wheel to rotate. 
        state = SwerveModuleState.optimize(state, getRotationState()); 
        // kPhysical ~ 3.5 m/s
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxDriveSpeedMetersPerSecond); // Give us a percentage speed for the spark to go to.  - Make sure it doesn't cross 1!
        
        double turningSpeed = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        // Limit the speed if past 100%, or 1. 
        turningSpeed = turningSpeed > 1 ? 1.0 : turningSpeed;
        turningMotor.set(ControlMode.PercentOutput, turningSpeed);  

        // Add debug info. 
        SmartDashboard.putString("Swerve[" + driveMotor.getChannel() + "] state:", state.toString()); // Give us the module debug info. .
        SmartDashboard.putNumber("Swerve[" + driveMotor.getChannel() + "] Speed ", state.speedMetersPerSecond / DriveConstants.kPhysicalMaxDriveSpeedMetersPerSecond); 
        SmartDashboard.putNumber("Swerve[" + driveMotor.getChannel() + "] turn", state.angle.getDegrees()); 
        SmartDashboard.putData("Swerve[" + driveMotor.getChannel() + "] PID Controller ", turningPidController); 

        SmartDashboard.putNumber("Swerve[" + driveMotor.getChannel() + "] Drive Input", state.speedMetersPerSecond / DriveConstants.kPhysicalMaxDriveSpeedMetersPerSecond); 
        SmartDashboard.putNumber("Swerve[" + driveMotor.getChannel() + "] Turn Input", turningSpeed);         
    }

    public void stopMotors(){
        // driveMotor.set(0);
        driveMotor.stopMotor();
        turningMotor.set(ControlMode.PercentOutput, 0);
    }


}
