/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autonomous.Control.AutoDriveControl;
import frc.robot.Autonomous.Pathing.Command.PathingCommand;
import frc.robot.Autonomous.Pathing.Iterative.TrajectoryFollow;
import frc.robot.Communication.Dashboard.Dashboard;
import frc.robot.Hardware.Electrical.PDP;
import frc.robot.Hardware.Joysticks.Xbox;
import frc.robot.Hardware.Sensors.NavX;
import frc.robot.Subsystems.ArcShooter;
import frc.robot.Subsystems.DriveTrainSystem;
import frc.robot.Utilities.Control.PIDF;
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

  private AutoDriveControl autoControl;

  private boolean hasRunDrive = false;
  private boolean hasTurned = false;

  private TrajectoryFollow pathing;

  // Command based trajectory
  private Command pathCommand;
  private PathingCommand pathingCommand;

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

    pathingCommand = new PathingCommand(drive);

    //Create a new shooter object
    shooter = new ArcShooter();

    autoControl = new AutoDriveControl(drive);

    // Used to make button interaction easier
    teleop = new TeleopControl();    

    //Clears sticky faults at robot start
    PDP.clearStickyFaults();
    
    //Reset the encoder positions at start
    drive.resetEncoders();

  }

  /**
   * Called every 20ms while the robot is turned on
   */
  @Override
  public void robotPeriodic() {
    //Update dashboard information
    updateDashboard();

    //Allow for commands to be scheduled
    CommandScheduler.getInstance().run();
  }

  /**
   * Called when the robot first enters the autonomous mode
   */
  @Override
  public void autonomousInit() {

    drive.resetEncoders(); 
    NavX.get().getAhrs().zeroYaw();  
    pathingCommand.resetOdometry();

    pathCommand = pathingCommand.getPathCommand();

    if(pathCommand != null && !pathCommand.isScheduled()){
      pathCommand.schedule();
    }
    
    hasRunDrive = false;
    hasTurned = false;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // if(!hasRunDrive){
    //   hasRunDrive = autoControl.DriveDistance(1);
    // }
    // else if (hasRunDrive && !hasTurned){
    //   hasTurned = autoControl.TurnToAngle(90);
    // }
  }

  /**
   * Called when the robot first enters the TeleOp period
   */
  @Override
  public void teleopInit() {

    //Allows for raw telop input (subject to change)
    drive.disableRampRate();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //Control the robot drive train
    drive.arcadeDrive(primary.rightStickX(), primary.leftStickY());

    //Toggle the shooters run status
   // teleop.runOncePerPress(primary.rightBumper(), () -> shooter.toggleShooter());
    
    shooter.manualShooter(primary.rightTrigger(), primary.leftTrigger());

    //Update subsystems
    //subsystemUpdater();
    
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
      Dashboard.setTable("Subsystems");

      //Add options to the dashboards
      Dashboard.createEntry("Fly-Wheel-RPM", 0.0);
      Dashboard.createEntry("Fly-Wheel-Speed-Status", false);

      //Create entry for the shooter current draw
      Dashboard.createEntry("Shooter-Current-Draw");

      //Switch the table to robot info
      Dashboard.setTable("SmartDashboard");

      //Create entries for the left side drive train current draw
      Dashboard.createEntry("Left-Side-Current-Draw");
      Dashboard.createEntry("Right-Side-Current-Draw");

      //Create entries for drive train encoders
      Dashboard.createEntry("Left-Side-Encoder");
      Dashboard.createEntry("Right-Side-Encoder");

      //Create entries to display graphs of temperature for the drive train
      Dashboard.createEntry("Left-Side-Temperature");
      Dashboard.createEntry("Right-Side-Temperature");

      //Add the NavX to the dashboard
      Dashboard.createEntry("Gyro");

      ///RAMSETE Specific Outputs

      Dashboard.setTable("RAMSETE");
      Dashboard.createEntry("Target-Left-Wheel-Speed", 0);
      Dashboard.createEntry("Target-Right-Wheel-Speed", 0);

      //The speeds that we want to reach
      Dashboard.createEntry("Left-Speed-Setpoint", 0);
      Dashboard.createEntry("Right-Speed-Setpoint", 0);

      // Output to the motors
      Dashboard.createEntry("Left-Wheel-Output", 0);
      Dashboard.createEntry("Right-Wheel-Output", 0);

      // Add X and Y as well as angle to the dashboard
      Dashboard.createEntry("Translational-Pose-X", 0);
      Dashboard.createEntry("Translational-Pose-Y", 0);
      Dashboard.createEntry("Rotational-Pose", 0);

  }

  /**
   * Meant to update values that dont have accessible update spots
   */
  private void updateDashboard(){

    Dashboard.setTable("SmartDashboard");
    
    // Add the values to the shuffle board in graph form
    Dashboard.setValue("Left-Side-Current-Draw", drive.getLeftSideCurrentDraw());
    Dashboard.setValue("Right-Side-Current-Draw", drive.getRightSideCurrentDraw());

    //Update the values on the dashboard for the drive train encoders
    Dashboard.setValue("Left-Side-Encoder", drive.getLeftSideEncoder());
    Dashboard.setValue("Right-Side-Encoder", drive.getRightSideEncoder());

    //Sets the temperatures for the drive train
    Dashboard.setValue("Left-Side-Temperature", drive.getLeftSideTemp());
    Dashboard.setValue("Right-Side-Temperature", drive.getRightSideTemp());

    //Update the navX angle on the dashboard
    Dashboard.setValue("Gyro", NavX.get().getAhrs());
  }
}
