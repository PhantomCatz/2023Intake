package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

public class CatzIntake {

  private Thread intakeThread;
  private final double INTAKE_THREAD_PERIOD      =0.02;
  
  public CatzLog data;

//-----------------------------------------------------------------------------------------------------------
//Roller
//-----------------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakeRollerMotor;

  private final int INTAKE_ROLLER_MC_ID        = 11; 

  private final double INTAKE_ROLLER_MOTOR_POWER       = -1.0;
  private final double OUTTAKE_ROLLER_MOTOR_POWER      =  1.0;
  private final double INTAKE_MOTOR_POWER_OFF          =  0.0;

  public final int INTAKE_ROLLER_OFF = 0;  //TBD to see if still needed
  public final int INTAKE_ROLLER_IN  = 1;
  public final int INTAKE_ROLLER_OUT = 2;
  public final int INTAKE_ROLLER_UNINITIALIZED = -999;

  public int   intakeRollerState = INTAKE_ROLLER_OFF;

//-----------------------------------------------------------------------------------------------------------
//Pivot
//-----------------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakePivotMotor;

  private final double INTAKE_PIVOT_REL_ENCODER_CPR = 2048.0;
  
  private final int INTAKE_PIVOT_MC_ID        = 10; 

  private final int INTAKE_UP_SLOT   = 0;
  private final double PID_INTAKE_UP_KP = 0.08; 
  private final double PID_INTAKE_UP_KI = 0.00;
  private final double PID_INTAKE_UP_KD = 0.00;

  private final int INTAKE_DOWN_SLOT = 1;
  private final double PID_INTAKE_DOWN_KP = 0.035;
  private final double PID_INTAKE_DOWN_KI = 0.00;
  private final double PID_INTAKE_DOWN_KD = 0.00;

  private final double INTAKE_PIVOT_PINION_GEAR = 11.0;
  private final double INTAKE_PIVOT_SPUR_GEAR   = 56.0;  
  private final double INTAKE_PIVOT_GEAR_RATIO  =INTAKE_PIVOT_SPUR_GEAR/INTAKE_PIVOT_PINION_GEAR;

  private final double INTAKE_PIVOT_SPROCKET_1  = 16.0;
  private final double INTAKE_PIVOT_SPROCKET_2  = 56.0;
  private final double INTAKE_PIVOT_SPROCKET_RATIO  = INTAKE_PIVOT_SPROCKET_2/INTAKE_PIVOT_SPROCKET_1;
  
  private final double INTAKE_PIVOT_FINAL_RATIO = INTAKE_PIVOT_GEAR_RATIO*INTAKE_PIVOT_SPROCKET_RATIO;


  private double deploymentMotorRawPosition;

  //Pivot status
  private  final int INTAKE_PIVOT_MODE_NULL                = 0;
  private  final int INTAKE_PIVOT_MODE_DEPLOY              = 1;
  private  final int INTAKE_PIVOT_MODE_DEPLOY_CALC         = 10;
  private  final int INTAKE_PIVOT_MODE_FULLY_DEPLOYED      = 2;
  private  final int INTAKE_PIVOT_MODE_INITIALIZATION      = 3;
  private  final int INTAKE_PIVOT_MODE_STOW_CALC           = 14;
  private  final int INTAKE_MODE_STOW_HOLD                 = 5;

//Pivot IntakeMode initialization
  private int intakePivotMode = INTAKE_PIVOT_MODE_NULL;

  private final static double INTAKE_STOWED_ANGLE   =  0.0;
  private final static double INTAKE_DEPLOYED_ANGLE = 89.0; //TBD angle need correct

  private final static double INTAKE_STOW_INITIAL_ANGLE   = INTAKE_DEPLOYED_ANGLE;
  private final static double INTAKE_DEPLOY_INITIAL_ANGLE = INTAKE_STOWED_ANGLE;

  private final double INTAKE_DEPLOY_SET_POSITION_START_ANGLE = 25.0;

  private final double INTAKE_DEPLOYED_ANGLE_COUNTS   = -(INTAKE_DEPLOYED_ANGLE * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);
  private final double INTAKE_STOWED_ANGLE_COUNTS     =  (INTAKE_STOWED_ANGLE   * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);
  
  private final double INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS = (5.0 * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);
  private final double INTAKE_SOFTLIMIT_MAX_ANGLE_COUNTS = INTAKE_DEPLOYED_ANGLE_COUNTS - INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS;
  private final double INTAKE_SOFTLIMIT_MIN_ANGLE_COUNTS = INTAKE_STOWED_ANGLE_COUNTS   + INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS;

  private final double INTAKE_PIVOT_DEPLOY_POWER = -0.2;
 
  private final int INTAKE_PIVOT_STOWED = 0; // MAY DELETE
  private final int INTAKE_PIVOT_DEPLOYED = 1;
  private final int INTAKE_PIVOT_IN_TRANSIT = 2;
  private final int INTAKE_PIVOT_UNINITIALIZED = -999;
  private int intakePivotState = INTAKE_PIVOT_STOWED;

  public Timer pivotTimer;

 
  public static double time=Timer.getFPGATimestamp();

  public static double finalMotorPower = 0;
  public static double Kp = 0.01;
  public static double Kd = 0.001;

  public final double INTAKE_DEPLOYMENT_TIME = 0.26;
  public static double targetAngle=0;
  public static double targetAngularRate = 0;
  public static double deltaAngle = 0;



  private final static double INTAKE_STOW_REDUCE_POWER_ANGLE  = 55.0;

  //coeffients

  public final double COEFF1 = 10;
  public final double COEFF2 = -15;
  public final double COEFF3 = 6;
  public final double INTAKE_MAX_TORCUE = 5.84;
  public final  double B_DEPLOY = (INTAKE_DEPLOYED_ANGLE-INTAKE_DEPLOY_INITIAL_ANGLE)/INTAKE_DEPLOYMENT_TIME;;
  public final double A3_DEPLOY = COEFF1*B_DEPLOY/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  public final double A4_DEPLOY = COEFF2*B_DEPLOY/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  public final double A5_DEPLOY = COEFF3*B_DEPLOY/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;

  public final double B_STOW = (INTAKE_STOWED_ANGLE-INTAKE_STOW_INITIAL_ANGLE)/INTAKE_DEPLOYMENT_TIME;
  public final double A3_STOW = COEFF1*B_STOW/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  public final double A4_STOW = COEFF2*B_STOW/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;  
  public final double A5_STOW = COEFF3*B_STOW/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  
  public static double powerForMotor;
  public static double angleDot = 0;
  public static double angleOld = 0;
  public static double currentAngle = 0;
  public static double timeOld = 0;
  public static double deltaTime = 0;

  public final double DEG2RAD = Math.PI/180.0;
  public final double INTAKE_INERTIA = 0.61;  //kg * m^2 
  public final double ALPHA3_DEPLOY = (A3_DEPLOY*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  public final double ALPHA4_DEPLOY = (A4_DEPLOY*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  public final double ALPHA5_DEPLOY = (A5_DEPLOY*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;

  public final double ALPHA3_STOW = (A3_STOW*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  public final double ALPHA4_STOW = (A4_STOW*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  public final double ALPHA5_STOW = (A5_STOW*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;

  public static boolean firstTimeThrough = true;

  private SupplyCurrentLimitConfiguration EncCurrentLimit;

  private final double  CURRENT_LIMIT_AMPS            = 60.0;
  private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
  private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
  private final boolean ENABLE_CURRENT_LIMIT          = true;

//---------------------------------------------definitions part end--------------------------------------------------------------
  
  public CatzIntake() {
    //need add softlimits

    intakeRollerMotor = new WPI_TalonFX(INTAKE_ROLLER_MC_ID);
    intakePivotMotor = new WPI_TalonFX(INTAKE_PIVOT_MC_ID);

    EncCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

    intakeRollerMotor.configFactoryDefault();
    intakeRollerMotor.setNeutralMode(NeutralMode.Coast);
    intakeRollerMotor.configSupplyCurrentLimit(EncCurrentLimit);


    intakePivotMotor.configFactoryDefault();
    intakePivotMotor.setNeutralMode(NeutralMode.Brake);
    intakePivotMotor.configSupplyCurrentLimit(EncCurrentLimit);
  
    intakePivotMotor.config_kP(INTAKE_UP_SLOT, PID_INTAKE_UP_KP);
    intakePivotMotor.config_kI(INTAKE_UP_SLOT, PID_INTAKE_UP_KI);
    intakePivotMotor.config_kD(INTAKE_UP_SLOT, PID_INTAKE_UP_KD);

    intakePivotMotor.config_kP(INTAKE_DOWN_SLOT, PID_INTAKE_DOWN_KP);
    intakePivotMotor.config_kI(INTAKE_DOWN_SLOT, PID_INTAKE_DOWN_KI);
    intakePivotMotor.config_kD(INTAKE_DOWN_SLOT, PID_INTAKE_DOWN_KD);

    intakePivotMotor.configForwardSoftLimitThreshold(INTAKE_SOFTLIMIT_MIN_ANGLE_COUNTS);
    intakePivotMotor.configReverseSoftLimitThreshold(INTAKE_SOFTLIMIT_MAX_ANGLE_COUNTS);

    intakePivotMotor.configForwardSoftLimitEnable(true);
    intakePivotMotor.configReverseSoftLimitEnable(true);

    pivotTimer = new Timer();
   
    intakeControl();

  }


//---------------------------------------------------Roller--------------------------------------------------------
   

  public void intakeRollerIn()
  {
      intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_ROLLER_MOTOR_POWER);
      intakeRollerState = INTAKE_ROLLER_IN;
  }

  public void intakeRollerOut()
  {
      intakeRollerMotor.set(ControlMode.PercentOutput,OUTTAKE_ROLLER_MOTOR_POWER);
      intakeRollerState = INTAKE_ROLLER_OUT;
  }


  public void intakeRollerOff()
  {
      intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_OFF);
      intakeRollerState = INTAKE_ROLLER_OFF;
  }

    


//------------------------------------------------Pivot--------------------------------------------------------------
    
    public void intakeControl()
    {
      System.out.println("mthod intakecontrol in");
      intakeThread = new Thread(() ->
      { 
        while(true)
        {
          System.out.println("+");
          //int currentAngle = (int)getIntakeDeployPositionDegrees();
          switch(intakePivotMode)
          {
              case INTAKE_PIVOT_MODE_NULL:
                
              break;
               
              case INTAKE_PIVOT_MODE_DEPLOY:

                currentAngle = getIntakeDeployPositionDegrees();
                if(currentAngle > INTAKE_DEPLOY_SET_POSITION_START_ANGLE )
                {
                  intakePivotMotor.set(ControlMode.Position, INTAKE_DEPLOYED_ANGLE_COUNTS);
                  //intakePivotMode = INTAKE_PIVOT_MODE_NULL;
                 // intakePivotMotor.set(ControlMode.MotionMagic, INTAKE_PIVOT_DEPLOY_ROTATION);
                }
                
              break;
              
              /* 
               case INTAKE_PIVOT_MODE_DEPLOY_CALC:
                System.out.println("in deploycalc");
                if(firstTimeThrough == true )
                {
                  pivotTimer.reset();
                  firstTimeThrough = false;
                }
                    
                currentAngle = getIntakeDeployPositionDegrees();
                time = pivotTimer.get();

                deltaAngle = currentAngle - angleOld;
                if(time > 0.01)
                {
                  deltaTime = time - timeOld;
                }
                else
                {
                  deltaTime = 100;//this is for initial time
                }
                angleOld = getIntakeDeployPositionDegrees();
                timeOld = time;
                angleDot = deltaAngle/deltaTime;
                powerForMotor = ALPHA3_DEPLOY * Math.pow(time , 3) - ALPHA4_DEPLOY *Math.pow(time , 4) + ALPHA5_DEPLOY *Math.pow(time , 5);
                
                targetAngle = (A3_DEPLOY*Math.pow(time , 3)) + (A4_DEPLOY*Math.pow(time , 4)) + (A5_DEPLOY*Math.pow(time , 5));
                targetAngularRate = (3 * A3_DEPLOY * Math.pow(time , 2)) + (4 * A4_DEPLOY * Math.pow(time , 3)) + (5 * A5_DEPLOY * Math.pow(time , 4));
                finalMotorPower = powerForMotor + Kp*(targetAngle - getIntakeDeployPositionDegrees()) + Kd*(targetAngularRate - deltaAngle/deltaTime); 
                finalMotorPower = -finalMotorPower;
                System.out.println("in deploy" + finalMotorPower);
                intakePivotMotor.set(ControlMode.PercentOutput,finalMotorPower);
                  
                currentAngle = getIntakeDeployPositionDegrees();
                if(currentAngle > INTAKE_DEPLOY_FINAL_ANGLE)
                {
                  firstTimeThrough = true;
                  timeOld = 0;
                  intakePivotMode = INTAKE_PIVOT_MODE_FULLY_DEPLOYED;
                }   
              break;
                
              case INTAKE_PIVOT_MODE_FULLY_DEPLOYED:
                intakePivotMotor.set(0);
              break;
*/
              
              case INTAKE_PIVOT_MODE_STOW_CALC:
                System.out.println("in stowcalc");

                if(firstTimeThrough == true)
                {
                  pivotTimer.reset();
                  firstTimeThrough = false;
                }

                currentAngle = getIntakeDeployPositionDegrees();
                time = pivotTimer.get();
                deltaAngle = currentAngle - angleOld;
                if(time > 0.01)
                {
                  deltaTime = time - timeOld;
                }
                else
                {
                  deltaTime = 100;// this is for initial time/ to provent a s
                }
                angleOld = getIntakeDeployPositionDegrees();
                timeOld = time;//Math.pow(4,2)
                angleDot = deltaAngle/deltaTime;
                powerForMotor = ALPHA3_STOW * (Math.pow(time , 3)) - ALPHA4_STOW *(Math.pow(time , 4)) + ALPHA5_STOW *(Math.pow(time , 5)); 
                targetAngle = (A3_STOW*Math.pow(time , 3)) + (A4_STOW*Math.pow(time , 4)) + (A5_STOW*Math.pow(time , 5));
                targetAngularRate = (3 * A3_STOW * Math.pow(time , 2)) + (4 * A4_STOW * Math.pow(time , 3)) + (5 * A5_STOW * Math.pow(time , 4));
                finalMotorPower = powerForMotor + Kp*(targetAngle - getIntakeDeployPositionDegrees()) + Kd*(targetAngularRate - deltaAngle/deltaTime); 
                finalMotorPower = -finalMotorPower;
                System.out.println("finalMotor Power:" + finalMotorPower);

                intakePivotMotor.set(ControlMode.PercentOutput,finalMotorPower);
                currentAngle = getIntakeDeployPositionDegrees();
                if(currentAngle < INTAKE_STOW_REDUCE_POWER_ANGLE)
                {
                  firstTimeThrough = true;
                  timeOld = 0;
                  intakePivotMode = INTAKE_MODE_STOW_HOLD;
                } 
              break;

              case INTAKE_MODE_STOW_HOLD:
                intakePivotMotor.set(0);
              break;

              default:
                intakePivotMotor.set(INTAKE_MOTOR_POWER_OFF);
                intakeRollerOff();
              break;
          }//eng of switch

          if(DataCollection.getLogDataID()== DataCollection.LOG_ID_INTAKE)
          {
            data = new CatzLog(Robot.currentTime.get(),
            targetAngularRate, targetAngle, finalMotorPower, powerForMotor, currentAngle, deltaAngle, deltaTime,
              -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0,
            DataCollection.boolData);  
            Robot.dataCollection.logData.add(data);
          }

          Timer.delay(INTAKE_THREAD_PERIOD);

        }//end of while true
              
      });
      intakeThread.start();
    
    }//end of intakeControl();


    /*-----------------------------------------------------------------------------------------
    *  
    * intakePivotDeploy()
    *
    *----------------------------------------------------------------------------------------*/
    public void intakePivotDeploy()
    {
      System.out.println("method deploy");
      intakePivotMotor.set(ControlMode.PercentOutput,INTAKE_PIVOT_DEPLOY_POWER);
      intakePivotMode = INTAKE_PIVOT_MODE_DEPLOY;
      
     //intakePivotMotor.selectProfileSlot(INTAKE_UP_SLOT, 0);
     //intakePivotMotor.set(ControlMode.Position,INTAKE_PIVOT_DEPLOY_ROTATION);
      
    }

    public void intakePivotStow()
    {
      intakePivotMode = INTAKE_PIVOT_MODE_STOW_CALC;
    }




    /*-----------------------------------------------------------------------------------------
    * getIntakeDeployPositionDegrees
    *----------------------------------------------------------------------------------------*/
    public double getIntakeDeployPositionDegrees(){
        deploymentMotorRawPosition = intakePivotMotor.getSelectedSensorPosition();

        double motorShaftRevolution = deploymentMotorRawPosition / INTAKE_PIVOT_REL_ENCODER_CPR;
        double pivotShaftRevolution = motorShaftRevolution / INTAKE_PIVOT_FINAL_RATIO;
        double pivotAngle =pivotShaftRevolution * -360.0;//motor  spin forward is positive 
        
        return pivotAngle;
       
    }


    /*-----------------------------------------------------------------------------------------
    *  
    * Smart Dashboard
    *
    *----------------------------------------------------------------------------------------*/
    public void smartDashboardIntake()
    {
        SmartDashboard.putNumber("PivotAngle", getIntakeDeployPositionDegrees());
        //SmartDashboard.putBooleanArray("yep",intakePivotMotor.)
       
    }

    public void smartDashboardIntake_Debug()
    {
      SmartDashboard.putNumber("PivotCounts", deploymentMotorRawPosition);
      SmartDashboard.putNumber("getClosedLoopError", intakePivotMotor.getClosedLoopError());
      SmartDashboard.putNumber("getClosedLoopTarget", intakePivotMotor.getClosedLoopTarget());
      SmartDashboard.putNumber("getStatorCurrent", intakePivotMotor.getStatorCurrent());
      
    }

  
}