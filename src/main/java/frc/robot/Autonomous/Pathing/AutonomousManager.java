package frc.robot.Autonomous.Pathing;

import frc.robot.Autonomous.Pathing.Enums.AutoPaths;
import frc.robot.Subsystems.ArcShooter;
import frc.robot.Subsystems.BallSystem;
import frc.robot.Subsystems.DriveTrainSystem;
import frc.robot.Utilities.Control.LimelightAlignment;

/**
 * The Autonomous Manager Class Handles All Control Of The Robot During The Autonomous Period
 * Each Auto Routine separated into classes
 * 
 * @author Will Richards
 */
public class AutonomousManager{

    // Vision

    // Class used to handle alignment from the limelight
    private LimelightAlignment alignment;

    // Whether or not the limelight is currently set to track a target
    private boolean limelightTracking = false;

    // Whether or not the robot has finished aligning with the target
    private boolean robotAligned = false;

    // Whether or not we are actively running the shooter
    private boolean runningShooter = false;

    //Pathing

    private Pathing pathing;

    //Subsystems
    
    private BallSystem ballSystem;
    private DriveTrainSystem drive;
    private ArcShooter shooter;

    //Auto Paths
    private EightBallOne basicEight;

    // The path the robot will run
    private AutoPaths path;



    /**
     * Initialize all required classes locally 
     * @param alignment limelight control loop
     * @param pathing RAMSETE command loop
     * @param ballSystem ball storage / indexing system
     * @param drive drive train system
     * @param shooter shooter system
     */
    public AutonomousManager(LimelightAlignment alignment, Pathing pathing, BallSystem ballSystem, DriveTrainSystem drive, ArcShooter shooter){

        // Init all variables
        this.alignment = alignment;
        this.pathing = pathing;
        this.ballSystem = ballSystem;
        this.drive = drive;
        this.shooter = shooter;

        // Create the first auto path
        basicEight = new EightBallOne();
    }

    /**
     * Returns a reference to the basic eight ball auto path
     * @return 8 ball auto path
     */
    public EightBallOne getBasicEight(){
        return basicEight;
    }

    /**
     * Set the robot path to run
     * @param path enum of the type
     */
    public void setPath(AutoPaths path){
        this.path = path;
    }

    /**
     * Overall setup that will run whatever the selected path is 
     */
    public void setup(){
        switch(path){
            case BASIC_EIGHT:
                getBasicEight().setup();
                break;
        }
    }

    /**
     * Overall periodic method that will run the selected auto paths periodic method
     */
    public void periodic(){
        switch(path){
            case BASIC_EIGHT:
                getBasicEight().periodic();
                break;
        }
    }
   

    /**
     * First simple auto path for eight balls
     */
    public class EightBallOne{

        /**
         * Called in the autonomous init function to setup the required parts of the routine
         */
        public void setup(){

            // Reset all characteristics of the robot on init
            pathing.resetProperties();

            // Robot is no longer aligned
            robotAligned = false;

            // Shooter is no longer running
            runningShooter = false;

            // Stop the indexer
            ballSystem.getIndexer().stopIndexing();

            // Stop the shooter
            shooter.stopShooter();

            // Starts the first section of the path and tells the robot to start tracking the target when complete
            pathing.runPath(PathContainer.basicEightPartOne(), () -> nextStage(()->setTracking(true)));

            
        }

        /**
         * Called during the autonomous periodic method to allow for active control 
         */
        public void periodic(){

            // Initial tracking statement
            if(getTrackingStatus()){
                alignShoot();
            }

        }

        /**
         * Allows the robot to align and shoot balls into the goal
         */
        private void alignShoot(){

            // Order is important so that the control loop doesn't run if the robot is already aligned
            if(!robotAligned && alignment.controlLoop()){
                robotAligned = true;
            }

            // Check if the robot is already running the shooter, if not start it
            else if(!runningShooter){
                shooter.enableShooter();
                runningShooter = true;
            }

            // Finally check if the robot is aligned at the shooter is at "Full speed", if so start the belts
            else if(robotAligned == true && shooter.isFull()){
                ballSystem.getIndexer().standardIndex();
            }

            // Update the shooter toggle
            subsystemUpdater();
        }
    }

     /**
     * Set whether or not the limelight should be trying to track something
     * @param value the tracking value
     */
    private void setTracking(boolean value){
        limelightTracking = value;
    }

    /**
     * Get whether or not the limelight is set to be tracking the target
     * @return the tracking status as a boolean
     */
    private boolean getTrackingStatus(){
        return limelightTracking;
    }

    /**
     * Used to update the statues of subsystems so they have constant feedback
     */
    private void subsystemUpdater(){
        shooter.runShooter();
    }

    /**
     * Is Called at the end of an auto path and it stops the robot and calls the next function in the path
     * @param nextFuntion the function to call
     */
    private void nextStage(Runnable nextFuntion){

        //Stop the robot
        drive.arcadeDrive(0, 0);

        // Run the next function
        nextFuntion.run();
    }

}