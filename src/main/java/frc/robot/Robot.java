/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autonomous.Basic.BasicAuto;
import frc.robot.Autonomous.Control.AutoDriveControl;
import frc.robot.Autonomous.Pathing.AutonomousManager;
import frc.robot.Autonomous.Pathing.PathContainer;
import frc.robot.Autonomous.Pathing.Pathing;
import frc.robot.Autonomous.Pathing.PathingCommand;
import frc.robot.Communication.Dashboard.Dashboard;
import frc.robot.Hardware.Electrical.PDP;
import frc.robot.Hardware.Joysticks.Xbox;
import frc.robot.Hardware.Pneumatics.AdvancedCompressor;
import frc.robot.Hardware.Sensors.NavX;
import frc.robot.Subsystems.ArcShooter;
import frc.robot.Subsystems.BallSystem;
import frc.robot.Subsystems.DriveTrainSystem;
import frc.robot.Subsystems.SwitchClimber;
import frc.robot.Utilities.Control.LimelightAlignment;
import frc.robot.Utilities.Teleop.TeleopControl;

/**
 * This is the main class where the robot control loop occurs
 */
public class Robot extends TimedRobot {

  // Joysticks / UI variables
  private Xbox primary;
  private Xbox secondary;

  // Subsystems
  private DriveTrainSystem drive;
  private ArcShooter shooter;
  private TeleopControl teleop;
  private BallSystem ballSystem;
  private SwitchClimber climber;

  //Auto
  private Pathing pathing;
  private LimelightAlignment alignment;
  private AutonomousManager autoManager;
  private AutoDriveControl autoDriveControl;

  private BasicAuto basicAuto;

  // Temp-Auto
  private boolean tracking = false;
  private boolean hasAligned = false;
  private boolean runningShooter = false;
  private int alignCount = 0;

  private boolean hasBackedUp = false;

  /**
   * Called as soon as the Robo-Rio boots, use like a constructor
   */
  @Override
  public void robotInit() {

    //Setup the dashboard
    DashboardSetup();

    // Assign the primary joystick to the correct port
    primary = new Xbox(new XboxController(0));
    
    // Secondary controller
    secondary = new Xbox(new XboxController(1));

    //Create a new shooter object
    shooter = new ArcShooter();

    ballSystem = new BallSystem();

    // Used to make button interaction easier
    teleop = new TeleopControl();   

    // Used to control the drive train of the robot
    drive = new DriveTrainSystem(ballSystem);

    // Climber used to climb the generator switch
    climber = new SwitchClimber();

    // Overall pathing class used to load paths and run them
    pathing = new Pathing(drive);

    // Uses the limelight to align the robot to the goal
    alignment = new LimelightAlignment(drive);

    // Create a new over all auto manager
    autoManager = new AutonomousManager(alignment, pathing, ballSystem, drive, shooter);

    autoDriveControl = new AutoDriveControl(drive);

    basicAuto = new BasicAuto(alignment, autoDriveControl, shooter, ballSystem);

    //Clears sticky faults at robot start
    PDP.clearStickyFaults();
    

    NavX.get().reset();
    NavX.get().resetYaw();

    pathing.resetProperties();
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

    // Stop running the compressor during auto
    AdvancedCompressor.stopCompressor();

    NavX.get().reset();

    drive.resetEncoders();

    drive.enableOpenRampRate(0.5);

    shooter.stopShooter();
    ballSystem.getIndexer().stopIndexing();

    basicAuto.runRendezvousFiveSetup();

   
    //pathing.runPath(PathContainer.basicEightPartOne());

    // Initializes the first path and resets everything
    //autoManager.getBasicEight().setup();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {  
    
    // Runs all required periodic functions
   //autoManager.getBasicEight().periodic();

    basicAuto.runRendezvousFive();
   
  }

  /**
   * Called when the robot first enters the TeleOp period
   */
  @Override
  public void teleopInit() {

    //Allows for raw telop input (subject to change)

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //Control the robot drive train and tracking
    if(!tracking){
      drive.arcadeDrive(primary.rightStickX(), primary.leftStickY());
      drive.disableOpenRampRate();
    }
    else{
      if(Math.abs(primary.rightStickX()) > 0.1 || Math.abs(primary.leftStickY()) > 0.1)
        tracking = false;
      
      alignment.controlLoop();
      drive.enableClosedRampRate(0.03);
    }

    //Toggle the shooters run status
    //teleop.runOncePerPress(secondary.leftBumper(), () -> shooter.toggleShooter());

    // If left trigger run winchs
    if(secondary.rightTrigger() > 0.1){
      climber.reverseWinch(secondary.rightTrigger());
    }
    else{
      climber.manualWinch(0);
    }

    // if(secondary.rightTrigger() > 0.1){
    //   climber.runRightWinch(secondary.rightTrigger());
    // }
    // else{
    //   climber.manualWinch(0);
    // }
    
    // Track the target
    teleop.runOncePerPress(primary.A(), () -> tracking = true);

    // Run until the compressor is full
    AdvancedCompressor.runUntilFull();

    // Temp. shooter control
    shooter.manualShooter(secondary.leftTrigger());

    // When A is pressed run the intake
    if(secondary.dPadUp())
      teleop.pressed(secondary.dPadUp(), () -> ballSystem.getIntake().runFrontIntakeBack(), () -> ballSystem.getIntake().stopFrontIntake());
    else
      teleop.pressed(secondary.dPadDown(), () -> ballSystem.getIntake().runFrontIntakeForward(), () -> ballSystem.getIntake().stopFrontIntake());

     //teleop.runOncePerPress(primary.dPadRight(), () -> ballSystem.getIntake().stopFrontIntake());
     //teleop.runOncePerPress(primary.dPadLeft(), () -> ballSystem.getIntake().runFrontIntakeBack());

    //When X is pressed attempt to index the balls into the shooter
    teleop.pressed(secondary.X(), () -> ballSystem.getIndexer().standardIndex(), () -> ballSystem.getIndexer().stopIndexing());

    //Extend and retract intake
     teleop.runOncePerPress(secondary.A(), () -> ballSystem.getIntake().extendIntake());
     teleop.runOncePerPress(secondary.B(), () -> ballSystem.getIntake().retractIntake());

    // teleop.runOncePerPress(primary.dPadLeft(), () -> climber.deployClimber());
    // teleop.runOncePerPress(primary.dPadRight(), () -> climber.retractClimber());

    // When both DpadUp and X are pressed climb
    if (secondary.dPadRight() && secondary.Y()){
      climber.deployClimber();
    }

    // //Update subsystems
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

    //AdvancedCompressor.runUntilFull();
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
      Dashboard.createEntry("Fly-Wheel-Total", 0.0);

      SmartDashboard.putBoolean("BallsInHopper", true);

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

      Dashboard.createEntry("Belt-Current", 0.0);
      Dashboard.createEntry("FrontIndexer-Current", 0.0);
      Dashboard.createEntry("BackIndexer-Current", 0.0);

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

      //Graphs of where the robot is supposed to be 
      Dashboard.createEntry("Expected-State-X", 0);
      Dashboard.createEntry("Expected-State-Y", 0);

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

    Dashboard.setValue("Belt-Current", ballSystem.getIndexer().getBeltCurrent());
    Dashboard.setValue("FrontIndexer-Current", ballSystem.getIndexer().getFrontIndexerCurrent());
    Dashboard.setValue("BackIndexer-Current", ballSystem.getIndexer().getBackIndexerCurrent());

    //Update the navX angle on the dashboard
    Dashboard.setValue("Gyro", NavX.get().getAhrs());
  }

  private void stopRobotAndRunMethod(Runnable action){
    drive.arcadeDrive(0, 0);
    action.run();
  }
}
