/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous.Pathing.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Hardware.Sensors.NavX;
import frc.robot.Subsystems.DriveTrainSystem;
import frc.robot.Utilities.Control.PID;

/**
 * Turns the robot 180 degrees
 * 
 * @author Will Richards
 */
public class Turn180 extends CommandBase {

  private PID pid;
  private DriveTrainSystem drive;

  //Turned is whether or not the command has completed, and the turnLoopCount makes sure the bot is actually at the right angle
  private boolean turned;
  private int turnLoopCount = 0;

  public Turn180(DriveTrainSystem drive) {

    this.drive = drive;

     // Turning Constant
     pid = new PID(.016,0,0.01);
     pid.setAcceptableRange(0.25);
     pid.setMaxOutput(0.2);
  }

  /**
   * Initialize the command and reset values
   */
  @Override
  public void initialize() {

    // Based off the current angle turn around
    if(NavX.get().getAngle() > 90){
      pid.setSetpoint(0);
    }
    else{
      pid.setSetpoint(180);
    }
    
    // Enable the ramp rate and reset variables
    drive.enableOpenRampRate(1);
    turnLoopCount = 0;
    turned = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Power to apply to the robot to turn
    double power = pid.calcOutput(NavX.get().getAngle());

    // Check if the robots output power is less than 0.26 motor power if so apply an additional power of 0.3 on top of the current power
    if(Math.abs(power) < 0.26){
        power += Math.copySign(0.3, power);
    }

    //If the PID is in range of being done stop the robot and check if the loop count is within an acceptable range
    if(pid.isInRange()){
        drive.arcadeDrive(0, 0);
        drive.enableOpenRampRate(0);
        if(turnLoopCount == 5)
            turned = true;
        else
            turnLoopCount++;
    }

    //Apply the PID output to the robot
    else{
        drive.arcadeDrive(power, 0);
        turned = false;
    }
  }

  /**
   * Called when the command is finished
   * Stops the robot and disables the ramp rate to not mess with auto
   */
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turned;
  }
}
