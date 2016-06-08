
package org.usfirst.frc.team5712.robot;

//Command based imports
import org.usfirst.frc.team5712.robot.subsystems.*;
import org.usfirst.frc.team5712.robot.commands.*;
import org.usfirst.frc.team5712.robot.OI;

//Default Robot imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//JavaCV imports
import static org.bytedeco.javacpp.opencv_core.*;
import static org.bytedeco.javacpp.opencv_imgproc.*;
import static org.bytedeco.javacpp.opencv_highgui.*;
import static org.bytedeco.javacpp.opencv_imgcodecs.*;

//OpenCV imports
import org.opencv.core.Core;

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
		
		/*
		*All code after this point (in the RobotInt() function) is for vision testing
		*/
		
		//This loads an image to the variable 'img'
		IplImage img = cvLoadImage("Test.png"); //The directory for images starts in the projects workspace in eclipse
		
		//These are the images that are the result of image processing
		IplImage hsvimg = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3); //Makes an image that is the same size as 'img', with an 8 bit resolution, with 3 channels since HSV images have 3 channels
		IplImage grayimg = cvCreatImage(cvGetSize(img), IPL_DEPTH_8U, 1); //Same as above except it uses one channel because grayscale images only use one channel per pixel
		
		cvCvtColor(img, hsvimg, CV_BGR2HSV); //'img' is the image that is the templae for color conversion, hsvimg is the image that the new image will be stored to
		cvCvtColor(img, grayimg, CV_BGR2GRAY); //same as above but instead of converting to HSV it is converting it to grayscale
		
		cvShowImage("Original Image", img); //This line displays the image in a new window on the PC (if you have this running on the PC I am not sure if it works if this is runnign on the roboRio)
		cvShowImage("HSV Image", img);
		cvShowImage("Grayscale Image", img);
		//cvWaitKey(); //uncomment this if the windows are closing right after they open
		
		cvReleaseImage(img);//This dealcolates data to this image (basically deletes it)
		cvReleaseImage(hsvimg);
		cvReleaseImage(grayimg);
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
