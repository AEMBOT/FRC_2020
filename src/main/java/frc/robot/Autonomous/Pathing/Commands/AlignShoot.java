/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autonomous.Pathing.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArcShooter;
import frc.robot.Subsystems.BallSystem;
import frc.robot.Subsystems.DriveTrainSystem;
import frc.robot.Utilities.Control.LimelightAlignment;

/**
 * Aligns the shooter and shoots all the balls in the hopper
 */
public class AlignShoot extends CommandBase {

  // Alignment based 
  private int alignLoopCount;
  private boolean robotAligned;
  private boolean runningShooter;

  //Whether or not there are still balls left in the hopper
  private boolean ballsInHopper;

  // Required classes to run systems
  private LimelightAlignment alignment;
  private ArcShooter shooter;
  private BallSystem ballSystem;
  private DriveTrainSystem drive;

  /**
   * Creates a new AlignShoot.
   */
  public AlignShoot(LimelightAlignment alignment, ArcShooter shooter, BallSystem ballSystem, DriveTrainSystem drive) {
    this.alignment = alignment;
    this.shooter = shooter;
    this.ballSystem = ballSystem;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.arcadeDrive(0, 0);
    alignLoopCount = 0;
    robotAligned = false;
    runningShooter = false;

    // Should be set later from the dashboard
    ballsInHopper = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
       // Order is important so that the control loop doesn't run if the robot is already aligned
       if(!robotAligned && alignment.controlLoop()){
        if(alignLoopCount == 10){
            robotAligned = true;            
        }
        else
        alignLoopCount++;
    }

    // // Check if the robot is already running the shooter, if not start it
    else if(!runningShooter){
        shooter.enableShooter();
        System.out.println(shooter.getStatus());
      
        
    }

    // Finally check if the robot is aligned at the shooter is at "Full speed", if so start the belts
    else if(robotAligned && shooter.isFull() && ballsInHopper){
        ballSystem.getIndexer().standardIndex();
        alignLoopCount = 0;
    }

    // Run the current status
    shooter.runShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
    ballSystem.getIndexer().stopIndexing();
    shooter.stopShooter();
  }

  // Returns true when all balls are gone and the robot is aligned to the target
  @Override
  public boolean isFinished() {
      if (robotAligned && !ballsInHopper){
        return true;
      }
      return false;
  }
}
