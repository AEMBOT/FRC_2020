package frc.robot.Autonomous.Basic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autonomous.Control.AutoDriveControl;
import frc.robot.Autonomous.Pathing.Enums.AutoPaths;
import frc.robot.Hardware.Sensors.NavX;
import frc.robot.Subsystems.ArcShooter;
import frc.robot.Subsystems.BallSystem;
import frc.robot.Utilities.Control.LimelightAlignment;

/**
 * Basic auto holder for running autos
 * 
 * @author Will Richards
 */
public class BasicAuto{

    LimelightAlignment alignment;
    AutoDriveControl autoDriveControl;

    ArcShooter shooter;
    BallSystem ballSystem;

    // Basic Back Auto
    private boolean hasBackedUp = false;
    private boolean hasAligned = false;
    private int alignCount = 0;

    // 5 Ball Rendezvous Zone Auto
    private boolean hasDroppedIntake = false;
    private boolean hasRaisedIntake = false;
    private boolean hasTurned = false;

    private Timer shooterTimeout;
    

    public BasicAuto(LimelightAlignment alignment, AutoDriveControl autoDriveControl, ArcShooter shooter, BallSystem ballSystem){
        this.alignment = alignment;
        this.autoDriveControl = autoDriveControl;

        this.shooter = shooter;
        this.ballSystem = ballSystem;

        shooterTimeout = new Timer();
    }

    /**
     * Sets up the auto and initilizes variables
     */
    public void setupAuto(AutoPaths autoPaths){

    }

    /**
     * Run the actual auto path
     */
    public void runAuto(AutoPaths autoPaths){

    }

    /**
     * Init and reset values
     */
    public void runBasicBackStartup(){
        hasBackedUp = false;
        hasAligned = false;
        alignCount = 0;
        shooterTimeout.reset();
    }

    /**
     * Run the basic 3 path
     */
    public void runBasicBack(){
        if(!hasBackedUp)
        hasBackedUp = autoDriveControl.DriveDistance(-2.1);
       else if(!hasAligned){
        if(alignment.controlLoop())
          alignCount++;
        else{
          alignCount--;
        }
    
        if(alignCount>=7)
          hasAligned = true;
       }
       else if(hasAligned && shooterTimeout.get() < 5){
         shooter.enableShooter();
         if(shooter.isFull()){
           ballSystem.getIndexer().standardIndex();
           System.out.println(shooterTimeout.get());
           if(shooterTimeout.get() <= 0)
            shooterTimeout.start();
         }
       }
       else if(shooterTimeout.get() > 5){
         shooter.stopShooter();
         ballSystem.getIndexer().stopIndexing();
       }
    
       shooter.runShooter();
    }

    /**
     * Setup for rendezvous
     */
    public void runRendezvousFiveSetup(){
        hasBackedUp = false;
        hasDroppedIntake = false;
        hasRaisedIntake = false;
        hasAligned = false;
        alignCount = 0;
        NavX.get().reset();
        shooterTimeout.reset();
    }

    /**
     * Drive pickup 2 from randezvous and then shoot 5
     */
    public void runRendezvousFive(){
        if(!hasDroppedIntake){
            ballSystem.getIntake().extendIntake();
            ballSystem.getIntake().runFrontIntakeForward();
            hasDroppedIntake = true;
        }
        else if(!hasBackedUp)
            hasBackedUp = autoDriveControl.DriveDistance(1.6, 0.35, 0.1);
        else if(!hasRaisedIntake){
            ballSystem.getIntake().retractIntake();
            ballSystem.getIntake().stopFrontIntake();
            hasRaisedIntake = true;
        }
        else if(!hasTurned){
            hasTurned = (alignment.limelight.getValidTarget() > 0);
            if(hasTurned){
              autoDriveControl.drive.arcadeDrive(0, 0);
            }
            else{
              autoDriveControl.drive.arcadeDrive(-0.45, 0);
            }
        }
        else if(!hasAligned){
            if(alignment.controlLoop())
                alignCount++;
            else
                alignCount--;

            if(alignCount>=7)
                hasAligned = true;
            
        }
        else if(hasAligned && shooterTimeout.get() < 5){
            shooter.enableShooter();
            if(shooter.isFull()){
              ballSystem.getIndexer().standardIndex();
              if(shooterTimeout.get() <= 0)
               shooterTimeout.start();
            }
          }
          else if(shooterTimeout.get() > 5){
            shooter.stopShooter();
            ballSystem.getIndexer().stopIndexing();
          }
       
          shooter.runShooter();

            
        
    }
}