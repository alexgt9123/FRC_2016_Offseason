package org.usfirst.frc.team5712.robot.subsystems;

import org.usfirst.frc.team5712.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveSubsystem extends Subsystem {
	
	public VictorSP leftFront, leftRear, rightFront, rightRear;
	
	public RobotDrive drive;
	
	Encoder leftDriveEncoder, rightDriveEncoder;
	
	public AHRS gyro;
	SerialPort serial_port;
		
	byte update_rate_hz = 50;
	public double degreesTurn;
	
	public DriveSubsystem(){
		
		leftFront = new VictorSP(RobotMap.LEFT_FRONT_MOTOR);
		leftRear = new VictorSP(RobotMap.LEFT_REAR_MOTOR);
		rightFront = new VictorSP(RobotMap.RIGHT_FRONT_MOTOR);
		rightRear = new VictorSP(RobotMap.RIGHT_REAR_MOTOR);
		
		drive = new RobotDrive(leftFront, leftRear, rightFront, rightRear);
		
		leftDriveEncoder = new Encoder(RobotMap.LEFT_DRIVE_ENCODER_A, RobotMap.LEFT_DRIVE_ENCODER_B, false, Encoder.EncodingType.k4X);
		rightDriveEncoder = new Encoder(RobotMap.RIGHT_DRIVE_ENCODER_A, RobotMap.RIGHT_DRIVE_ENCODER_B, false, Encoder.EncodingType.k4X);
		
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
		
		gyro = new AHRS(SerialPort.Port.kMXP);
		gyro.reset();
					
	}
	
	public void display(){
		SmartDashboard.putNumber("Encoder (Left Drive)", leftDriveEncoder.get());
		SmartDashboard.putNumber("Encoder (Right Drive)", rightDriveEncoder.get());
		SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
		SmartDashboard.putNumber("Gyro X", gyro.getDisplacementX());
		SmartDashboard.putNumber("Gyro Y", gyro.getDisplacementY());
	}
	
	public void resetDriveEncoders(){
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
	}
	
	public void resetGyro(){
		gyro.reset();
	}
	
    public void initDefaultCommand() {
    }
    
    public void turn150degrees(){
    	
    	if (gyro.getYaw() > -120) {
			//turn robot until yaw is between -145 and -150
			leftFront.set(-0.4 + (-120 - gyro.getYaw())/180); 
			leftRear.set(0.4 + (-120 - gyro.getYaw())/180);
			rightFront.set(-0.4 + (-120 - gyro.getYaw())/180);
			rightRear.set(-0.4 + (-120 - gyro.getYaw())/180);
		}
		else if(gyro.getYaw() < -123) {
			//turns the robot back if desired angle is passed
			leftFront.set(0.4 - (-120 - gyro.getYaw())/180); 
			leftRear.set(-0.4 - (-120 - gyro.getYaw())/180);
			rightFront.set(0.4 - (-120 - gyro.getYaw())/180);
			rightRear.set(0.4 - (-120 - gyro.getYaw())/180);
		}
		else if(gyro.getYaw() < -120 && gyro.getYaw() > -123) {
			//stop robot when yaw is between -145 and -150
			leftFront.set(0);
			leftRear.set(0);
			rightFront.set(0);
			rightRear.set(0);
		}
    }
    
    public void turnXdegrees(double degreesTurn){
    	if (gyro.getYaw() > -degreesTurn ) {
    		leftFront.set(-0.4 + (-degreesTurn - angle)/180); 
			leftRear.set(0.4 + (-degreesTurn - gyro.getYaw())/180);
			rightFront.set(-0.4 + (-degreesTurn - gyro.getYaw())/180);
			rightRear.set(-0.4 + (-degreesTurn - gyro.getYaw())/180);
		}
    	else if (gyro.getYaw() < (-degreesTurn - 3)){
    		leftFront.set(0.4 - (-degreesTurn - gyro.getYaw())/180); 
			leftRear.set(-0.4 - (-degreesTurn - gyro.getYaw())/180);
			rightFront.set(0.4 - (-degreesTurn - gyro.getYaw())/180);
			rightRear.set(0.4 - (-degreesTurn - gyro.getYaw())/180);
    	}
    }
    
    
}


