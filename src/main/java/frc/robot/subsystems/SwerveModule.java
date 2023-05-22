package frc.robot.subsystems;

import javax.swing.SwingWorker;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
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

    // Constructor where we update everything we need to create a module. 
    public SwerveModule(int SparkPort, int TalonId, boolean driveMotorReversed, boolean turningMotorReversed){
        // Define motors to be those we created with passed in ports. 
        driveMotor = new Spark(SparkPort);
        turningMotor = new TalonSRX(TalonId);

        // Set motors to be inversed if we told it to. 
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
    
        turningPidController = new PIDController(0.1, 0, 0); // Double check value. 


    };
}
