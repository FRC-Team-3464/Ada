package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class SwerveModule {
    private final Spark driveMotor; // Change to Spark
    private final TalonSRX turningMotor; // Change to Talon

    // Don't have encoders for the drive motor
    // private final SensorCollection turningEncoder = turningMotor.getSensorCollection(); // Specify later to be just quadraty - Might not need. 

    private final PIDController turningPidController; // Add a controller for our turning motor. 
    
    // We're basically screwed with this one
    // private final AnalogInput absoluteEncoder;
    // private final boolean absoluteEncoderReversed; // Are we reversed. 
    // private final double absoluteEncoderOffsetRad; // Motor offset in Radians - probably not used. 


    private static final double turnEncoderRatio = (71/1); // Gear ratio for PG71 ~ 71
    private static final double wheelGearRatio = 0.25;



    private static final double turnMaxRPM = 75;
  

    // Constructor where we update everything we need to create a module. 
    public SwerveModule(int SparkPort, int TalonId, boolean driveMotorReversed, boolean turningMotorReversed){
        // Define motors to be those we created with passed in ports. 
        driveMotor = new Spark(SparkPort);
        turningMotor = new TalonSRX(TalonId);

        // Set motors to be inversed if we told it to. 
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
    
        turningPidController = new PIDController(0.1, 0, 0); // Double check value. 
        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // Basically controller moves to find the shortest path to a target in a circle - a circle's diameter is 2pi. 
        
        // Will need to make that constant but can't use for the encoder. 
        // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        resetEncoders(); // Call our method to reset our encoder when we create the module. 
    };

    public double getTurningPosition(){
        // Return Talon Turning Position
        // Gear RATIO IS WRONG
        return turningMotor.getSelectedSensorPosition(0) * turnEncoderRatio; // pidId 0 is simply the quadratic encoder - can be set with the phoenix tuner. 
    }

    public double getTurningVelocity(){
        // Return Talon Turning Position
        // Gear RATIO IS WRONG
        return turningMotor.getSelectedSensorVelocity(0) * turnEncoderRatio; // pidId 0 is simply the quadratic encoder - can be set with the phoenix tuner. 
    }

    public void resetEncoders() {
        // NOTE: The ms means that the code will throw an error after desired ms - at 100 ms in this example. 
        turningMotor.setSelectedSensorPosition(0, 0,100);  // pidId 0 is simply the quadratic encoder - can be set with the phoenix tuner. 
    }

    // // Create a rotation 2d that symbolizes the 2d rotation state of the wheel. 
    // public Rotation2d rotationState(){
    //     return new Rotation2d(get
    // }
}
