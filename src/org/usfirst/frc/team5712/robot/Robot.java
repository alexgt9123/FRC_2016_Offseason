
package org.usfirst.frc.team5712.robot;

import org.usfirst.frc.team5712.robot.subsystems.*;
import org.usfirst.frc.team5712.robot.commands.*;
import org.usfirst.frc.team5712.robot.OI;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * 
 * @author Team 5712
 * 
 */

public class Robot extends IterativeRobot {

	public static boolean IS_COMPETITION_ROBOT;
	
	public static DriveSubsystem driveSubsystem = new DriveSubsystem();	
	public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	public static CameraSubsystem cameraSubsystem = new CameraSubsystem();
	
	public static OI oi;
	
	//DriveStick commands
    Command invertMotorsFalseCommand, invertMotorsTrueCommand;
    Command shiftGearCommand;
    Command solenoidInCommand, solenoidOutCommand;
    Command switchCamCommand;
    Command turn135degreesCommand, turn150degreesCommand;
    
    //shootStick commands
    Command shootCommand;
    
    //Autonomous commands
    Command autonomousSelected;
    Command angleSelected;
    
    CommandGroup lowbarAutonomousCommandGroup, moatAutonomousCommandGroup;
    
    
    //Autonomous Selector
    SendableChooser autoChooser, angleChooser;
    
    public double shootTickGoal = 10 * -7.5; //tick to degree ratio (degrees/tick) * angle desired
    
    public void robotInit() {
		oi = new OI();
		
		autoChooser = new SendableChooser();
		autoChooser.addDefault("Lowbar", lowbarAutonomousCommandGroup);
		autoChooser.addObject("Moat", moatAutonomousCommandGroup);
		SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);
		
		angleChooser = new SendableChooser();
		angleChooser.addDefault("120", 120);
		angleChooser.addObject("150", 150);
		
		Robot.shooterSubsystem.compressor.setClosedLoopControl(true);
		
		cameraSubsystem.cameraInit();
		
		driveSubsystem.resetGyro();
		driveSubsystem.resetDriveEncoders();
		shooterSubsystem.resetShooterEncoder();
    }
	
    public void disabledInit(){

    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

    public void autonomousInit() {
        System.out.println("Autonomous Selected: " + autoChooser.getSelected());
        
    	Robot.shooterSubsystem.shooter.set(DoubleSolenoid.Value.kReverse);
        Robot.driveSubsystem.resetDriveEncoders();
        Robot.driveSubsystem.resetGyro();
        
        autonomousSelected = (Command) autoChooser.getSelected();
        autonomousSelected.start();
        
    }

    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        
        Robot.driveSubsystem.display();
        Robot.shooterSubsystem.display();
    }

    public void teleopInit() {
    	if (autonomousSelected != null) autonomousSelected.cancel();
    	
    	Robot.driveSubsystem.resetDriveEncoders();
    	Robot.driveSubsystem.resetGyro();
    	Robot.shooterSubsystem.resetShooterEncoder();
    	
		driveSubsystem.drive.arcadeDrive(oi.driveStick);
    }

    public void teleopPeriodic() {
    	Robot.driveSubsystem.display();
    	Robot.shooterSubsystem.display();

		//Robot.driveSubsystem.degreesTurn = (double) autoChooser.getSelected();
    }
    
    public void testPeriodic() {
    }
    
    public void displayCommandsOnDashboard() {
    	SmartDashboard.putData("Shoot Command", new shootCommand());
    	SmartDashboard.putData("Intake Command", new intakeCommand());
    	SmartDashboard.putData("Turn X Degrees Command", new turnXdegreesCommand());
    }
}
