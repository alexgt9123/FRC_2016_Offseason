#include <iostream>

class Robot : public IteritaveRobot
{
  private:
  class RobotDrive *drive;
  class Victor *frontLeft, *frontRight, *rearLeft, *rearRight;
  class Joystick *left, *right;
  
  void AutonomousInit()
  {
    
  }
  
  void AutonomousPeriodic()
  {
    
  }
  
  void TeleopInit()
  {
    frontLeft = new Victor(1);
    frontRight = new Victor(2);
    rearLeft = new Victor(3);
    rearRight = new Victor(4);
    drive = new RobotDrive(frontLeft, frontRight, rearLeft, rearRight);
    left = new Joystick(1);
    right = new Joystick(2);
  }
  
  void operatorControl()
  {
    while(IsOperatorControl() && IsEnabled())
    {
      drive->TankDrive(left, right);
      wait(0.01);
    }
  }
  
  int getRobotMode() //0 = Disabled, 1 = Enabled, 2 = Auto, 3 = Teleop, 4 = Test
  {
    if(IsDisabled())
    {
      return 0;
    }
    else if(IsEnabled())
    {
      return 1;
    }
    else if(IsAutonomous())
    {
      return 2;
    }
    else if(IsOperatorControl())
    {
      return 3;
    }
    else if(IsTest())
    {
      return 4;
    }
  }
  
  double getBatteryVoltage()
  {
    return DriverStation::GetInstance().GetBatteryVoltage();
  }
  
  int getAlliance() //1 = Blue, 2 = Red, 0 = None
  {
    DriverStation::Alliance color;
    color = DriverStation::GetInstance().GetAlliance();
    
    if(color == DriverStation::Alliance::kBlue)
    {
      return 1;
    }
    else if(color == DriverStation::Alliance::kRed)
    {
      return 2;
    }
    else
    {
      return 0;
    }
    
    double getMatchTime()
    {
      return DriverStation::GetInstance().GetMatchTime();
    }
  }
}
