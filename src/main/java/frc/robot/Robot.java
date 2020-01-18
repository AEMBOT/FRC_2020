/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Autonomous.Pathing.TrajectoryFollow;
import frc.robot.Communication.Dashboard.Dashboard;
import frc.robot.Hardware.Electrical.PDP;
import frc.robot.Hardware.Joysticks.Xbox;
import frc.robot.Hardware.Sensors.NavX;
import frc.robot.Subsystems.ArcShooter;
import frc.robot.Subsystems.DriveTrainSystem;
import frc.robot.Utilities.Teleop.TeleopControl;

/**
 * This is the main class where the robot control loop occurs
 */
public class Robot extends TimedRobot {

  // Joysticks / UI variables
  private Xbox primary;

  // Subsystems
  private DriveTrainSystem drive;
  private ArcShooter shooter;
  private TeleopControl teleop;

  // Autonomous Control
  private TrajectoryFollow pathing;

  /**
   * Called as soon as the Robo-Rio boots, use like a constructor
   */
  @Override
  public void robotInit() {

    //Setup the dashboard
    DashboardSetup();

    // Assign the primary joystick to the correct port
    primary = new Xbox(new XboxController(0));

    // Init the drive train system with the correct gamepad
    drive = new DriveTrainSystem();

    //Create a new shooter object
    shooter = new ArcShooter();

    // Used to make button interaction easier
    teleop = new TeleopControl();

   // pathing = new TrajectoryFollow(drive, "");

    //Clears sticky faults at robot start
    PDP.clearStickyFaults();

  }

  /**
   * Called every 20ms while the robot is turned on
   */
  @Override
  public void robotPeriodic() {
    //Update dashboard information
    updateDashboard();
  }

  /**
   * Called when the robot first enters the autonomous mode
   */
  @Override
  public void autonomousInit() {
    pathing.followPath();
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

    //Control the robot drive train
    drive.arcadeDrive(primary.leftStickY(), primary.rightStickX());

    //Toggle the shooters run status
    teleop.runOncePerPress(primary.rightBumper(), () -> shooter.toggleShooter());
    
    //Update subsystems
    subsystemUpdater();
    
    // Called to signify the end of one teleop loop to reset button properties,
    // don't delete
    teleop.endPeriodic();
  }

  /**
   * This function is called periodically during test mode, can be used to quickly
   * check systems, or maybe even a diagnostic program
   */
  @Override
  public void testPeriodic() {
      
  }

  /**
   * Will call update methods for subsystems so as to not clutter the teleopPeriodic method
   */
  private void subsystemUpdater(){
    shooter.runShooter();
  }

  /**
   * Used to setup the Shuffleboard
   */
  private void DashboardSetup(){

      //Set up the net table
      Dashboard.setTable("SmartDashboard");

      //Add options to the dashboards
      Dashboard.createEntry("Fly-Wheel-RPM", 0.0);
      Dashboard.createEntry("Fly-Wheel-Speed-Status", false);

      //Create entry for the shooter current draw
      Dashboard.createEntry("Shooter-Current-Draw");

      //Create entries for the left side drive train current draw
      Dashboard.createEntry("Left-Side-Current-Draw");
      Dashboard.createEntry("Right-Side-Current-Draw");

      Dashboard.createEntry("Gyro");
  }

  /**
   * Meant to update values that dont have accessible update spots
   */
  private void updateDashboard(){

    // Add the values to the shuffle board in graph form
    Dashboard.setValue("Left-Side-Current-Draw", drive.getLeftSideCurrentDraw());
    Dashboard.setValue("Right-Side-Current-Draw", drive.getRightSideCurrentDraw()); 

    //Update the navX angle on the dashboard
    Dashboard.setValue("Gyro", NavX.get().getAhrs());
  }
}
