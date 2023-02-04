package frc.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.CatzDrivetrain;
import frc.DataLogger.CatzLog;
import frc.robot.*;

public class CatzAutonomous {
    final public static double NAVX_RESET_WAIT_TIME = 0.2;

    public boolean runningRadialTurn = false;
    public double turnRateRadians;

    /*******************************************************************************
	*  Autonomous - Drive Straight
	*******************************************************************************/
    public double currentEncCountRt;
    public double currentEncCountLt;
    public double rightInitialEncoderCnt;

    public double distanceAbs;
    public double distanceMoved;

    public Timer driveStraightTimer;

    private final double DRIVE_STRAIGHT_LOOP_PERIOD  = 0.02;
    
    private final double DRIVE_STRAIGHT_ACCEL_PERIOD = 0.2;
    private final int    DRIVE_STRAIGHT_ACCEL_STEPS  = (int)((DRIVE_STRAIGHT_ACCEL_PERIOD / DRIVE_STRAIGHT_LOOP_PERIOD) + 1);
    

    public final double STOP_THRESHOLD_DIST =  5.0;
    public final double DECEL_THRESHOLD_DIST = 60.0;
    public final double TIME_TO_SLOW_IN_SEC = 0.5; 


    public final double FPS_TO_INCHES_PER_100MS         = (( 1.0 / 10.0) * (12.0 /  1.0));    //10 100 msec samples per sec & 12 inches per foot

    private final double DS_MIN_VELOCITY_LIMIT_FPS      = 2.0;
    private  double dsMinVelLimitCntsPer100ms = 0.0;

    public double maxVelocityFPS               = 0.0;
    public double currentVelocityFPS;


    /***************************************************************************
	 * PID Turn Constants
	 ***************************************************************************/
    public static double PID_TURN_THRESHOLD   = 2.5;

	/***************************************************************************
	 * PID_TURN_DELTAERROR_THRESHOLD_HI - Delta Error Values larger than this are
	 * considered invalid and will be ignored 
     * PID_TURN_DELTAERROR_THRESHOLD_LO - When drivetrain power drops below the
     * PID_TURN_MIN_xxx_POWER, we will check to see if deltaError is below this 
     * threshold before setting power at PID_TURN_MIN_xxx_POWER.
	 ***************************************************************************/
	final public static double PID_TURN_DELTAERROR_THRESHOLD_HI = 4.0;
	final public static double PID_TURN_DELTAERROR_THRESHOLD_LO = 0.11;

	final public static double PID_TURN_FILTER_CONSTANT    = 0.7;
	      public static double PID_TURN_POWER_SCALE_FACTOR = 1.0;

	      public static double PID_TURN_KP = 0.08;
	      public static double PID_TURN_KI = 0.0;
	      public static double PID_TURN_KD = 0.012;

	final public static double PID_TURN_INTEGRAL_MAX =  1.0;
	final public static double PID_TURN_INTEGRAL_MIN = -1.0;

    // 0.4 is min power to move robot when it is stopped
	final public static double PID_TURN_MIN_POS_POWER = 0.6; 
    final public static double PID_TURN_MIN_NEG_POWER = -PID_TURN_MIN_POS_POWER;
    
    final public double TURN_MIN_VELOCITY_LIMIT_FPS       = 5.0;

    private static double PID_TURN_VELOCITY_FPS = 8.0;

	/***************************************************************************
	 * PID Turn Variables
	***************************************************************************/
	static Timer functionTimer;
    static Timer pdTimer;
    
    final public static double DRIVE_MAX_POS_POWER  =  1.0;
	final public static double DRIVE_MAX_NEG_POWER  = -1.0;

    private final double TURN_IN_PLACE_SCALE_FACTOR = 0.4474;


    static double pidTurnkP = PID_TURN_KP;
    static double pidTurnkI = PID_TURN_KI;
    static double pidTurnkD = PID_TURN_KD;

	static double currentError; 
	static double deltaError;
	static double derivative;
	static double deltaT;

	static double power;

	static double previousError;
	static double totalError;

	static double currentAngle;
	static double currentAngleAbs;
	static double targetAngle;
	static double targetAngleAbs;

	static double loopDelay = 0.005;

	static double previousDerivative = 0.0;

	static boolean tuningMode = false;
	static boolean debugMode  = false;

    private CatzLog data;

    private static double pidTurnDecelRate   = 0.5;
    private static double pidTurnDecelAngle  = 20.0;


    private final boolean INTAKE_ROLLER_ON  = true;
    private final boolean NO_INTAKE_ROLLING = false;

    private boolean autonShooterOn = false;

    public CatzAutonomous()
    {
        driveStraightTimer = new Timer();
    }


    public void TurnInPlace(double degreesToTurn, double speedPercentOut, double timeoutSeconds)
    {
        boolean turnInPlaceDone = false;

        double  currentTime       = 0.0;
        double  angleRemainingAbs = 999.0;

        Robot.navX.reset();
        Timer.delay(NAVX_RESET_WAIT_TIME);

        functionTimer = new Timer();
        functionTimer.reset();
        functionTimer.start(); 

        //setTurnValues(degreesToTurn);
        //Robot.drivetrain.rotateInPlace(0);

        currentAngle  = Robot.navX.getAngle();
        targetAngle   = degreesToTurn + currentAngle;
        currentError  = targetAngle   - currentAngle;

        //targetAngleAbs = Math.abs(targetAngle);  

        // data = new CatzLog( Robot.dataCollectionTimer.get(), currentAngle, currentError,
        //                     Robot.drivetrain.LT_FRNT_MODULE.getAngle(), Robot.drivetrain.LT_FRNT_MODULE.getError(), Robot.drivetrain.LT_FRNT_MODULE.getFlipError(),
        //                     Robot.drivetrain.LT_BACK_MODULE.getAngle(), Robot.drivetrain.LT_BACK_MODULE.getError(), Robot.drivetrain.LT_BACK_MODULE.getFlipError(),
        //                     Robot.drivetrain.RT_FRNT_MODULE.getAngle(), Robot.drivetrain.RT_FRNT_MODULE.getError(), Robot.drivetrain.RT_FRNT_MODULE.getFlipError(),
        //                     Robot.drivetrain.RT_BACK_MODULE.getAngle(), Robot.drivetrain.RT_BACK_MODULE.getError(), Robot.drivetrain.RT_BACK_MODULE.getFlipError(), -999);
        // Robot.dataCollection.logData.add(data);

        /*----------------------------------------------------------------------
        *  Set Initial Turn Velocity
        ----------------------------------------------------------------------*/
        if (degreesToTurn < 0.0) 
        {
            speedPercentOut = -speedPercentOut;
            System.out.println("target angle negative");
        }

        //Robot.drivetrain.rotateInPlace(speedPercentOut);

        while (turnInPlaceDone == false) 
        {
            System.out.println("turning: " + currentAngle);
            dataCollection();
            currentTime  = functionTimer.get();
            currentAngle = Robot.navX.getAngle();
    
            // calculates proportional term
            currentError      = targetAngle - currentAngle;
            angleRemainingAbs = Math.abs(currentError);

            //clean up PID TURN THRESHOLD without mulitply
            if (angleRemainingAbs < PID_TURN_THRESHOLD) 
            { 
                turnInPlaceDone = true;
                System.out.println("DONE");
            }

            else
            {
                if (currentTime > timeoutSeconds) 
                {
                    turnInPlaceDone = true;
                    System.out.println("DONEDONE");
                } else{
                    Robot.drivetrain.rotateInPlace(speedPercentOut);
                }
                // else
                // {
                //     if (angleRemainingAbs < pidTurnDecelAngle) 
                //     {
                //         Robot.drivetrain.rotateInPlace(speedPercentOut);
                //         System.out.println("decel");
                //     }
                // }
            }
        }

        /*----------------------------------------------------------------------
        *  We either hit target angle or have timed out - stop
        ----------------------------------------------------------------------*/
        speedPercentOut = 0.0;
        Robot.drivetrain.rotateInPlace(speedPercentOut);
        System.out.println("END OF TURNINPLACE");
    }

    public static void setTurnValues(double degreesToTurn) 
    {
        double degreesToTurnAbs;
    
        degreesToTurnAbs = Math.abs(degreesToTurn);
    
        if (degreesToTurnAbs <= 60.0) 
        {
            pidTurnDecelRate   = 0.98;
            pidTurnDecelAngle  = 10.0;
        }
        else if (degreesToTurnAbs <= 75.0) 
        {
            pidTurnDecelRate   = 0.98;
            pidTurnDecelAngle  = 12.0;
        }
        else if (degreesToTurnAbs <= 105.0) 
        {
            pidTurnDecelRate   = 0.98;
            pidTurnDecelAngle  = 14.0;
        }
        else if (degreesToTurnAbs <= 140.0) 
        {
            pidTurnDecelRate   = 0.98;
            pidTurnDecelAngle  = 16.0;
        }
        
        // degreesToTurnAbs > 140.0
        else 
        { 
            pidTurnDecelRate   = 0.98;
            pidTurnDecelAngle  = 18.0;
        }
    }   //End of setTurnValues()

    /***************************************************************************
    *
    * Autonomous Paths
    * 
    ***************************************************************************/
    public void testPath()
    {
        TurnInPlace(90.0, 0.25, 2.0);
        TurnInPlace(-90.0, 0.25, 2.0);
    }

    public void dataCollection()
    {
        if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_AUTON_TURN)
        {
            data = new CatzLog( Robot.dataCollectionTimer.get(), currentAngle, currentError,
                            Robot.drivetrain.LT_FRNT_MODULE.getAngle(), Robot.drivetrain.LT_FRNT_MODULE.getError(), Robot.drivetrain.LT_FRNT_MODULE.getFlipError(),
                            Robot.drivetrain.LT_BACK_MODULE.getAngle(), Robot.drivetrain.LT_BACK_MODULE.getError(), Robot.drivetrain.LT_BACK_MODULE.getFlipError(),
                            Robot.drivetrain.RT_FRNT_MODULE.getAngle(), Robot.drivetrain.RT_FRNT_MODULE.getError(), Robot.drivetrain.RT_FRNT_MODULE.getFlipError(),
                            Robot.drivetrain.RT_BACK_MODULE.getAngle(), Robot.drivetrain.RT_BACK_MODULE.getError(), Robot.drivetrain.RT_BACK_MODULE.getFlipError(), -999);
        Robot.dataCollection.logData.add(data);
        }
    }
}
