// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.CatzIntake;


public class Robot extends TimedRobot {
  final int XBOX_PORT_ID = 0;
  public static CatzIntake intake;
  private XboxController xboxDrv;
  public static boolean elevatorPivoted = false;

  @Override
  public void robotInit() {
    intake = new CatzIntake();
    xboxDrv = new XboxController(XBOX_PORT_ID);
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
    if (xboxDrv.getLeftStickButtonPressed()){
      
        intake.intakePivotDeploy();
      
      }
      else if(xboxDrv.getAButtonPressed()){
        intake.intakePivotStow();
      }
      
    
    
    
}

  

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

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
