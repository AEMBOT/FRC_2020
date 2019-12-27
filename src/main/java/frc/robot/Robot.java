/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Hardware.Joysticks.Xbox;
import frc.robot.Subsystems.DriveTrainSystem;

/**
 *  This is the main class where the robot control loop occurs
 */
public class Robot extends TimedRobot {

  //Drive train variable and joystick variable
  private DriveTrainSystem drive;
  private Xbox primary;

  /**
   * Called as soon as the Robo-Rio boots, use like a constructor
   */
  @Override
  public void robotInit() {

    //Assign the primary joystick to the correct port
    primary = new Xbox(new XboxController(0));

    //Init the drive train system with the correct gamepad
    drive = new DriveTrainSystem();
  }

  /**
   * Called every 20ms while the robot is turned on
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * Called when the robot first enters the autonomous mode
   */
  @Override
  public void autonomousInit() {
   
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * Called when the robot first enters the TeleOp period
   */
  @Override
  public void teleopInit() {
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode, can be used to quickly check systems, or maybe even a diagnostic program
   */
  @Override
  public void testPeriodic() {

  }
}
