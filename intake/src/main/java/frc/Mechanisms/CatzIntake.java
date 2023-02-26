package frc.Mechanisms;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

 
public class CatzIntake {

  private Thread intakeThread;
  
//-----------------------------------------------------------------------------------------------------------
//Roller
//-----------------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakeRollerMotor;
  private final int INTAKE_ROLLER_MC_ID        = 11; 

  private final double INTAKE_ROLLER_MOTOR_POWER       = 0.5;
  private final double OUTTAKE_ROLLER_MOTOR_POWER      = 0.5;
  private final double INTAKE_MOTOR_POWER_OFF          = 0.0;

  private final int INTAKE_ROLLER_OFF = 0;
  private final int INTAKE_ROLLER_IN  = 1;
  private final int INTAKE_ROLLER_OUT = 2;
  private final int INTAKE_ROLLER_UNINITIALIZED = -999;
  private int   intakeRollerState = INTAKE_ROLLER_OFF;

//-----------------------------------------------------------------------------------------------------------
//Pivot
//-----------------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakePivotMotor;
 // private WPI_TalonFX 
  private final int INTAKE_PIVOT_MC_ID        = 10; 

  private final int INTAKE_PIVOT_FULLY_DEPLOYED_ANGLE = 90;

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
  private  final int INTAKE_PIVOT_MODE_STOW                = 4;
  private  final int INTAKE_PIVOT_MODE_STOW_CALC           = 14;
  private  final int INTAKE_MODE_STOW_HOLD                 = 5;

//Pivot IntakeMode initialization
  private int intakePivotMode = INTAKE_PIVOT_MODE_NULL;

  public boolean intakeStowed = true;
  public boolean intakeDeployed = false;

  public int counter=0;

  private final double INTAKE_PIVOT_DEPLOY_POWER = 0.2;
  private final double INTAKE_PIVOT_STOW_POWER   = 0.4;
  private final double INTAKE_PIVOT_DEPLOY_POWER_OFF_ANGLE = 25.0;

  private final double INTAKE_THREAD_PERIOD      =0.02;

  private final int INTAKE_PIVOT_STOWED = 0;
  private final int INTAKE_PIVOT_DEPLOYED = 1;
  private final int INTAKE_PIVOT_IN_TRANSIT = 2;
  private final int INTAKE_PIVOT_UNINITIALIZED = -999;
  private int intakePivotState = INTAKE_PIVOT_STOWED;

  private Timer pivotTimer;

  private final double INTAKE_PIVOT_REL_ENCODER_CPR = 2048.0;

  private static double time=Timer.getFPGATimestamp();

  private static double finalMotorPower = 0;
  private static double Kp = 0.01;
  private static double Kd = 0.001;
  private final static int INTAKE_DEPLOY_FINAL_ANGLE = 90;
  private final static int INTAKE_DEPLOY_INITIAL_ANGLE = 0;
  private final int INTAKE_STOW_FINAL_ANGLE = 0;
  private final int INTAKE_STOW_INITIAL_ANGLE = 90;
  private final double INTAKE_DEPLOYMENT_TIME = 0.26;
  private static double targetAngle=0;
  private static double targetAngularRate = 0;
  private static double deltaAngle = 0;

  private final static double INTAKE_FULLY_DEPLOYED_ANGLE = 89.0;
  private final static double INTAKE_FULLY_STOWED_ANGELE  = 1.0;


  // a are the coeffient for the fifth order polynomial profile
  private static double a3;
  private static double a4;
  private static double a5;
  //coeffients

  private final double COEFF1 = 10;
  private final double COEFF2 = -15;
  private final double COEFF3 = 6;
  private final double INTAKE_MAX_TORCUE = 5.84;
  private static double angleDot = 0;
  private static double angleOld = 0;
  private static double currentAngle = 0;
  private static double timeOld = 0;
  private static double deltaTime = 0;
  private final  double B_DEPLOY = (INTAKE_DEPLOY_FINAL_ANGLE-INTAKE_DEPLOY_INITIAL_ANGLE)/INTAKE_DEPLOYMENT_TIME;;
  private final double A3_DEPLOY = COEFF1*B_DEPLOY/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  private final double A4_DEPLOY = COEFF2*B_DEPLOY/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  private final double A5_DEPLOY = COEFF3*B_DEPLOY/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;

  private final double B_STOW = (INTAKE_STOW_FINAL_ANGLE-INTAKE_STOW_INITIAL_ANGLE)/INTAKE_DEPLOYMENT_TIME;
  private final double A3_STOW = COEFF1*B_STOW/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  private final double A4_STOW = COEFF2*B_STOW/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;  
  private final double A5_STOW = COEFF3*B_STOW/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  private static double powerForMotor;

  private final double DEG2RAD = Math.PI/180.0;
  private final double INTAKE_INERTIA = 0.61;//kg * m^2 
  private final double ALPHA3_DEPLOY = (A3_DEPLOY*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  private final double ALPHA4_DEPLOY = (A4_DEPLOY*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  private final double ALPHA5_DEPLOY = (A5_DEPLOY*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;

  private final double ALPHA3_STOW = (A3_STOW*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  private final double ALPHA4_STOW = (A4_STOW*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  private final double ALPHA5_STOW = (A5_STOW*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;

  private static boolean firstTimeThrough = true;

 



//---------------------------------------------definition part end--------------------------------------------------------------
  
  public CatzIntake() {
    //need add softlimits

    intakeRollerMotor    = new WPI_TalonFX(INTAKE_ROLLER_MC_ID);
    intakePivotMotor = new WPI_TalonFX(INTAKE_PIVOT_MC_ID);

    intakeRollerMotor.configFactoryDefault();
    intakePivotMotor.configFactoryDefault();

    // intakeRoller.setSomething?current limit
    intakeRollerMotor.setNeutralMode(NeutralMode.Coast);

    //Timer pivotTimer = new Timer();

  
   //initialize for pivot motor
   //sensor-->0;

    intakePivotMotor.setNeutralMode(NeutralMode.Brake);

    intakeControl();

  }


//---------------------------------------------------Roller--------------------------------------------------------
   

  public void intakeRollerIn()
    {
        intakeRollerMotor.set(ControlMode.PercentOutput,-INTAKE_ROLLER_MOTOR_POWER);
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
    
    public void intakeControl(){
    
        intakeThread = new Thread(() ->
        { 
            while(true)
            {
                //int currentAngle = (int)getIntakeDeployPositionDegrees();
                switch(intakePivotMode)
                {
                    case INTAKE_PIVOT_MODE_NULL:
                      
                      break;

                    case INTAKE_PIVOT_MODE_DEPLOY:

                      currentAngle = getIntakeDeployPositionDegrees();
                      if(currentAngle > INTAKE_PIVOT_DEPLOY_POWER_OFF_ANGLE )
                      {
                        intakePivotMotor.set(INTAKE_MOTOR_POWER_OFF);
                        intakePivotMode = INTAKE_PIVOT_MODE_NULL;
                      }
                      else if(currentAngle<-5)
                      {
                        intakePivotMotor.set(INTAKE_MOTOR_POWER_OFF);
                      }
                      break;
                      
                    case INTAKE_PIVOT_MODE_STOW:
                      currentAngle = getIntakeDeployPositionDegrees();
                      if(currentAngle < 25.0)
                      {
                        intakePivotMotor.set(0.1);
                      }
                      else if(currentAngle < 2.0)
                      {
                        intakePivotMotor.set(INTAKE_MOTOR_POWER_OFF);
                      }

                      break;

                    case INTAKE_PIVOT_MODE_DEPLOY_CALC:
                  
                        if(firstTimeThrough == true ){
                        pivotTimer.reset();
                        firstTimeThrough = false;
                        }
                         
                        currentAngle = getIntakeDeployPositionDegrees();
                        time = pivotTimer.get();

                        deltaAngle = currentAngle - angleOld;
                        if(time > 0.01){
                        deltaTime = time - timeOld;
                         }else{
                        deltaTime = 100;//this is for initial time
                        }
                        angleOld = getIntakeDeployPositionDegrees();
                        timeOld = time;
                        angleDot = deltaAngle/deltaTime;
                        powerForMotor = ALPHA3_DEPLOY * Math.pow(time , 3) - ALPHA4_DEPLOY *Math.pow(time , 4) + ALPHA5_DEPLOY *Math.pow(time , 5);
                       
                        targetAngle = (A3_DEPLOY*Math.pow(time , 3)) + (A4_DEPLOY*Math.pow(time , 4)) + (A5_DEPLOY*Math.pow(time , 5));
                        targetAngularRate = (3 * A3_DEPLOY * Math.pow(time , 2)) + (4 * A4_DEPLOY * Math.pow(time , 3)) + (5 * A5_DEPLOY * Math.pow(time , 4));
                        finalMotorPower = powerForMotor + Kp*(targetAngle - getIntakeDeployPositionDegrees()) + Kd*(targetAngularRate - deltaAngle/deltaTime); 
                        intakePivotMotor.set(finalMotorPower);
                         
                        currentAngle = getIntakeDeployPositionDegrees();
                        if(currentAngle > INTAKE_DEPLOY_FINAL_ANGLE){
                          firstTimeThrough = true;
                          timeOld = 0;
                          intakePivotMode = INTAKE_PIVOT_MODE_FULLY_DEPLOYED;
                        }
                        
                        
                        break;

                    case INTAKE_PIVOT_MODE_FULLY_DEPLOYED:
                        intakePivotMotor.set(0);
                        break;

                    
                    case INTAKE_PIVOT_MODE_STOW_CALC:
                        
                      if(firstTimeThrough == true){
                        pivotTimer.reset();
                        firstTimeThrough = false;
                        }

                        
                        currentAngle = getIntakeDeployPositionDegrees();
                        time = pivotTimer.get();
                        deltaAngle = currentAngle - angleOld;
                        if(time > 0.01){
                        deltaTime = time - timeOld;
                        }else{
                        deltaTime = 100;// this is for initial time/ to provent a s
                        }
                        angleOld = getIntakeDeployPositionDegrees();
                        timeOld = time;//Math.pow(4,2)
                        angleDot = deltaAngle/deltaTime;
                        powerForMotor = ALPHA3_STOW * (Math.pow(time , 3)) - ALPHA4_STOW *(Math.pow(time , 4)) + ALPHA5_STOW *(Math.pow(time , 5)); 
                        targetAngle = (A3_STOW*Math.pow(time , 3)) + (A4_STOW*Math.pow(time , 4)) + (A5_STOW*Math.pow(time , 5));
                        targetAngularRate = (3 * A3_STOW * Math.pow(time , 2)) + (4 * A4_STOW * Math.pow(time , 3)) + (5 * A5_STOW * Math.pow(time , 4));
                        finalMotorPower = powerForMotor + Kp*(targetAngle - getIntakeDeployPositionDegrees()) + Kd*(targetAngularRate - deltaAngle/deltaTime); 
                        intakePivotMotor.set(finalMotorPower);
                        currentAngle = getIntakeDeployPositionDegrees();
                        if(currentAngle < INTAKE_FULLY_STOWED_ANGELE){
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

                Timer.delay(INTAKE_THREAD_PERIOD);

            }//eng of while true
                
        });
        intakeThread.start();
    
    }//end of intakeControl();


    /*-----------------------------------------------------------------------------------------
    *  
    * intakePivotDeploy()
    *
    *----------------------------------------------------------------------------------------*/
    public void intakePivotDeploy(){
      intakePivotMode = INTAKE_PIVOT_MODE_DEPLOY;

      intakePivotMotor.set(-INTAKE_PIVOT_DEPLOY_POWER);
      
    }

    public void intakePivotStow(){

      intakePivotMode = INTAKE_PIVOT_MODE_STOW;

      intakePivotMotor.set(INTAKE_PIVOT_STOW_POWER);

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
    
    }

  
}