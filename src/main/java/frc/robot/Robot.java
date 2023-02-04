// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.Autonomous.CatzAutonomous;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.CatzDrivetrain;
import frc.Mechanisms.PathSelection;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  public static CatzAutonomous auton;
  public static CatzDrivetrain drivetrain;
  public static CatzConstants constants;
  public static DataCollection datacollection;
  public static PathSelection pathSelection;

  public static AHRS navX;
  
  public static DataCollection dataCollection;
  public static Timer currentTime;
  public static Timer dataCollectionTimer;
  public ArrayList<CatzLog> dataArrayList;

  private XboxController xboxDrv;
  private double steerAngle = 0.0;
  private double drivePower = 0.0;
  private double turnPower = 0.0;
  private double gyroAngle = 0.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit()
  {
    xboxDrv = new XboxController(0);

    drivetrain = new CatzDrivetrain();
    dataCollection = new DataCollection();
    auton         = new CatzAutonomous();
    constants     = new CatzConstants();
    pathSelection = new PathSelection();

    navX = new AHRS();
    navX.reset();
    navX.setAngleAdjustment(-navX.getYaw());


    
    dataArrayList = new ArrayList<CatzLog>();
    dataCollection.dataCollectionInit(dataArrayList);
    

    dataCollectionTimer = new Timer();
    dataCollectionTimer.start();
    currentTime = new Timer();
    pathSelection.initializePathOptions();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    drivetrain.updateShuffleboard();
    dataCollection.updateLogDataID();
    SmartDashboard.putNumber("NavX", navX.getAngle());
    SmartDashboard.putNumber("Joystick", steerAngle);

    //drivetrain.testAngle();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit()
  {
    dataCollectionTimer.reset();
    dataCollectionTimer.start();
    drivetrain.setBrakeMode();
    pathSelection.determinePath();

    
    dataCollection.setLogDataID(DataCollection.LOG_ID_AUTON_TURN);
    dataCollection.startDataCollection();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic()
  {
    gyroAngle = getGyroAngle();
    //drivetrain.drive(0.0, 0.5, gyroAngle);
    
    if(drivePower >= 0.1)
    {
      auton.dataCollection();
    }
      
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() 
  {
    drivetrain.setBrakeMode();

    currentTime.reset();
    currentTime.start();

    dataCollection.startDataCollection();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic()
  {
    steerAngle = calcJoystickAngle(xboxDrv.getLeftX(), xboxDrv.getLeftY());
    drivePower = calcJoystickPower(xboxDrv.getLeftX(), xboxDrv.getLeftY());
    turnPower = xboxDrv.getRightX();
    gyroAngle = getGyroAngle();

    if(drivePower >= 0.1)
    {
      if(Math.abs(turnPower) >= 0.1)
      {
        drivetrain.translateTurn(steerAngle, drivePower, turnPower, gyroAngle);
      }
      else
      {
        drivetrain.drive(steerAngle, drivePower, gyroAngle);
      }
      drivetrain.dataCollection();
    }
    else if(Math.abs(turnPower) >= 0.1)
    {
      drivetrain.rotateInPlace(turnPower);
      drivetrain.dataCollection();
    }
    else
    {
      drivetrain.setSteerPower(0.0);
      drivetrain.setDrivePower(0.0);
    }

    if(xboxDrv.getStartButtonPressed())
    {
      zeroGyro();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit()
  {
    currentTime.stop();
    drivetrain.setCoastMode();
    
    if(dataCollection.logDataValues == true)
    {
      dataCollection.stopDataCollection();

      try 
      {
        dataCollection.exportData(dataArrayList);
      } 
      catch (Exception e) 
      {
        e.printStackTrace();
      }
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic()
  {

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit()
  {
    drivetrain.initializeOffsets(navX);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}



  public double calcJoystickAngle(double xJoy, double yJoy)
    {
        double angle = Math.atan(Math.abs(xJoy) / Math.abs(yJoy));
        angle *= (180 / Math.PI);

        if(yJoy <= 0)   //joystick pointed up
        {
            if(xJoy < 0)    //joystick pointed left
            {
                //no change
            }
            if(xJoy >= 0)   //joystick pointed right
            {
                angle = -angle;
            }
        }
        else    //joystick pointed down
        {
            if(xJoy < 0)    //joystick pointed left
            {
                angle = 180 - angle;
            }
            if(xJoy >= 0)   //joystick pointed right
            {
                angle = -180 + angle;
            }
        }
        return angle;
    }

    public double calcJoystickPower(double xJoy, double yJoy)
    {
      return (Math.sqrt(Math.pow(xJoy, 2) + Math.pow(yJoy, 2)));
    }

    public void zeroGyro()
    {
        navX.setAngleAdjustment(-navX.getYaw());
    }

    public double getGyroAngle()
    {
        return navX.getAngle();
    }
}
