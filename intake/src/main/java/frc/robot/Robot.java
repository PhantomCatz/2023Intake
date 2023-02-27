// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.CatzIntake;


public class Robot extends TimedRobot {
  public ArrayList<CatzLog> dataArrayList;
  public static DataCollection dataCollection;
  public static Timer currentTime;
  final int XBOX_PORT_ID = 0;
  public static CatzIntake intake;
  private XboxController xboxDrv;
  public static boolean elevatorPivoted = false;
double testKP = 0.0333333;
  @Override
  public void robotInit() {
    dataCollection = new DataCollection();
    intake = new CatzIntake();
    xboxDrv = new XboxController(XBOX_PORT_ID);
    currentTime = new Timer();

    dataArrayList = new ArrayList<CatzLog>();
    dataCollection.dataCollectionInit(dataArrayList);
  }

 
  @Override
  public void robotPeriodic() {
    intake.smartDashboardIntake();

    intake.smartDashboardIntake_Debug();

  }

  
  @Override
  public void autonomousInit() {
   
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
   
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
   
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    

  //----------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------Intake----------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------------------------------------
   
  //intake.updateIntakeSpeed();

  //leftTrigger-->control rollerOut
  
    if (xboxDrv.getLeftTriggerAxis() > 0.2)
    {
      intake.intakeRollerOut();    
      //intake.intakeRollerOut = true;
    }
    //rightTrigger-->control rollerIn
    else if (xboxDrv.getRightTriggerAxis() > 0.2)
    {      
      intake.intakeRollerIn();    
      //intake.intakeRollerStartIn = true;
    }
    else{
      intake.intakeRollerOff();
    }
    


    
    //LeftStickButton--> control Deploy
    if (xboxDrv.getBButtonPressed())
    {
      
        intake.intakePivotDeploy();
      
    }
    else if(xboxDrv.getAButtonPressed())
    {
        intake.intakePivotStow();
    }
    else if(xboxDrv.getStartButton()){
        if(xboxDrv.getRightY() > 0.2 || xboxDrv.getRightY() < -0.2){
          intake.intakePivotMotor.set(xboxDrv.getRightY() * 0.3);
        }
        else{
          intake.intakePivotMotor.set(0.0);
        }
    }

    if (xboxDrv.getBackButtonPressed())
    {
      System.out.println("KP IS NOW " + testKP);
      intake.intakePivotMotor.config_kP(0, testKP );
      intake.intakePivotMotor.config_kP(1,testKP );
      testKP = testKP * 2;

    }
    
    
}

  

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    currentTime.stop();
    
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
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
