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

  private boolean turned;
  private int turnLoopCount = 0;

  public Turn180(DriveTrainSystem drive) {

    this.drive = drive;

     // Turning Constant
     pid = new PID(.016,0,0.01);
     pid.setAcceptableRange(0.25);
     pid.setMaxOutput(0.2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(180);
    drive.enableOpenRampRate(1);
    turnLoopCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double power = pid.calcOutput(NavX.get().getAngle());

    // Check if the robots output power is less than 0.26 motor power if so apply an additional power of 0.3 ontop of the current power
    if(Math.abs(power) < 0.26){
        power += Math.copySign(0.3, power);
    }

    if(pid.isInRange()){
        drive.arcadeDrive(0, 0);
        drive.enableOpenRampRate(0);
        if(turnLoopCount == 5)
            turned = true;
        else
            turnLoopCount++;
    }
    else{
        drive.arcadeDrive(power, 0);
        turned = false;
    }
  }

  // Called once the command ends or is interrupted.
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
