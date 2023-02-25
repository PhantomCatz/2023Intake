package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;

 
public class CatzIntake {

  private Thread intakeThread;
  
//-----------------------------------------------------------------------------------------------------------
//Roller
//-----------------------------------------------------------------------------------------------------------
  //private  VictorSPX intakeRoller;
  private WPI_TalonFX intakeRollerMotor;
  //correction needed
  private final int INTAKE_ROLLER_MC_ID        = 11; 

  private final double INTAKE_ROLLER_MOTOR_POWER       = 0.5;
  private final double OUTTAKE_ROLLER_MOTOR_POWER      = 0.5;
  private final double INTAKE_MOTOR_POWER_OFF          = 0.0;

  public final int INTAKE_ROLLER_OFF = 0;
  public final int INTAKE_ROLLER_IN  = 1;
  public final int INTAKE_ROLLER_OUT = 2;
  public final int INTAKE_ROLLER_UNINITIALIZED = -999;
  public int   intakeRollerState = INTAKE_ROLLER_OFF;

//-----------------------------------------------------------------------------------------------------------
//Pivot
//-----------------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakePivotMotor;
  private final int INTAKE_PIVOT_MC_ID        = 10; 

  private final int INTAKE_PIVOT_FULLY_DEPLOYED_ANGLE = 90;

  private final double INTAKE_PIVOT_PINION_GEAR = 11.0;
  private final double INTAKE_PIVOT_SPUR_GEAR   = 56.0;  
  private final double INTAKE_PIVOT_GEAR_RATIO  =INTAKE_PIVOT_SPUR_GEAR/INTAKE_PIVOT_PINION_GEAR;

  private final double INTAKE_PIVOT_SPROCKET_1  = 16.0;
  private final double INTAKE_PIVOT_SPROCKET_2  = 56.0;
  private final double INTAKE_PIVOT_SPROCKET_RATIO  = INTAKE_PIVOT_SPROCKET_2/INTAKE_PIVOT_SPROCKET_1;
  
  private final double INTAKE_PIVOT_FINAL_RATIO = INTAKE_PIVOT_GEAR_RATIO*INTAKE_PIVOT_SPROCKET_RATIO;




  //Pivot status
  public  final int INTAKE_PIVOT_MODE_NULL                = 0;
  public  final int INTAKE_PIVOT_MODE_DEPLOY              = 1;
  public  final int INTAKE_PIVOT_MODE_FULLY_DEPLOYED      = 2;
  public  final int INTAKE_PIVOT_MODE_INITIALIZATION      = 3;
  public  final int INTAKE_PIVOT_MODE_STOW                = 4;
  public  final int INTAKE_MODE_STOW_HOLD           = 5;

//Pivot IntakeMode initialization
  public int intakePivotMode = INTAKE_PIVOT_MODE_NULL;

  public boolean intakeStowed = true;
  public boolean intakeDeployed = false;

  public int counter=0;

  public final int INTAKE_PIVOT_STOWED = 0;
  public final int INTAKE_PIVOT_DEPLOYED = 1;
  public final int INTAKE_PIVOT_IN_TRANSIT = 2;
  public final int INTAKE_PIVOT_UNINITIALIZED = -999;
  public int intakePivotState = INTAKE_PIVOT_STOWED;

  public Timer pivotTimer;

  public static double time=Timer.getFPGATimestamp();

  public static double finalMotorPower = 0;
  public static double Kp = 0.01;
  public static double Kd = 0.001;
  public final static int INTAKE_DEPLOY_FINAL_ANGLE = 90;
  public final static int INTAKE_DEPLOY_INITIAL_ANGLE = 0;
  public final int INTAKE_STOW_FINAL_ANGLE = 0;
  public final int INTAKE_STOW_INITIAL_ANGLE = 90;
  public final double INTAKE_DEPLOYMENT_TIME = 0.26;
  public static double targetAngle=0;
  public static double targetAngularRate = 0;
  public static double deltaAngle = 0;

  public final static double INTAKE_FULLY_DEPLOYED_ANGLE = 89.0;
  public final static double INTAKE_FULLY_STOWED_ANGELE  = 1.0;


  // a are the coeffient for the fifth order polynomial profile
  public static double a3;
  public static double a4;
  public static double a5;
  //coeffients

  public final double COEFF1 = 10;
  public final double COEFF2 = -15;
  public final double COEFF3 = 6;
  public final double INTAKE_MAX_TORCUE = 5.84;
  public static double angleDot = 0;
  public static double angleOld = 0;
  public static double currentAngle = 0;
  public static double timeOld = 0;
  public static double deltaTime = 0;
  public final  double B_DEPLOY = (INTAKE_DEPLOY_FINAL_ANGLE-INTAKE_DEPLOY_INITIAL_ANGLE)/INTAKE_DEPLOYMENT_TIME;;
  public final double A3_DEPLOY = COEFF1*B_DEPLOY/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  public final double A4_DEPLOY = COEFF2*B_DEPLOY/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  public final double A5_DEPLOY = COEFF3*B_DEPLOY/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;

  public final double B_STOW = (INTAKE_STOW_FINAL_ANGLE-INTAKE_STOW_INITIAL_ANGLE)/INTAKE_DEPLOYMENT_TIME;
  public final double A3_STOW = COEFF1*B_STOW/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  public final double A4_STOW = COEFF2*B_STOW/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;  
  public final double A5_STOW = COEFF3*B_STOW/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
  public static double powerForMotor;

  public final double DEG2RAD = Math.PI/180.0;
  public final double INTAKE_INERTIA = 0.61;//kg * m^2 
  public final double ALPHA3_DEPLOY = (A3_DEPLOY*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  public final double ALPHA4_DEPLOY = (A4_DEPLOY*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  public final double ALPHA5_DEPLOY = (A5_DEPLOY*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;

  public final double ALPHA3_STOW = (A3_STOW*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  public final double ALPHA4_STOW = (A4_STOW*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;
  public final double ALPHA5_STOW = (A5_STOW*DEG2RAD*INTAKE_INERTIA)/INTAKE_PIVOT_FINAL_RATIO/INTAKE_MAX_TORCUE;

  public static boolean firstTimeThrough = true;



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
                    case INTAKE_PIVOT_MODE_DEPLOY:
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
                       
                        powerForMotor = ALPHA3_DEPLOY * (time *time *time) - ALPHA4_DEPLOY *(time *time *time*time) + ALPHA5_DEPLOY *(time *time *time *time *time);
                        
                        targetAngle = (A3_DEPLOY*time*time*time) + (A4_DEPLOY*time*time*time*time) + (A5_DEPLOY*time*time*time*time*time);
                        targetAngularRate = (3 * A3_DEPLOY * time * time) + (4 * A4_DEPLOY * time *time * time) + (5 * A5_DEPLOY * time * time * time * time);
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

                    case INTAKE_PIVOT_MODE_STOW:
                        
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
                        powerForMotor = ALPHA3_STOW * (time *time *time) - ALPHA4_STOW *(time *time *time*time) + ALPHA5_STOW *(time *time *time *time *time); 
                        targetAngle = (A3_STOW*time*time*time) + (A4_STOW*time*time*time*time) + (A5_STOW*time*time*time*time*time);
                        targetAngularRate = (3 * A3_STOW * time * time) + (4 * A4_STOW * time *time * time) + (5 * A5_STOW * time * time * time * time);
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
                }
                
            }
                //switch()
        });
        intakeThread.start();
    
}



    public double getIntakeDeployPositionDegrees(){
        double deploymentMotorRawPosition = intakePivotMotor.getSelectedSensorPosition();
        return ((deploymentMotorRawPosition / 2048) *360 * INTAKE_PIVOT_FINAL_RATIO ) % 360;//deploymentMotorRawPosition 0-4096 is 1 rotation
        //% count will goes
        //
    }

  
}