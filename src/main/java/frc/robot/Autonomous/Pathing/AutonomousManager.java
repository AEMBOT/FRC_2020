package frc.robot.Autonomous.Pathing;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous.Pathing.Commands.AlignShoot;
import frc.robot.Autonomous.Pathing.Commands.Turn180;
import frc.robot.Autonomous.Pathing.Enums.AutoPaths;
import frc.robot.Hardware.Sensors.NavX;
import frc.robot.Subsystems.ArcShooter;
import frc.robot.Subsystems.BallSystem;
import frc.robot.Subsystems.DriveTrainSystem;
import frc.robot.Utilities.Control.LimelightAlignment;
import frc.robot.Utilities.Control.PID;

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


        private int alignLoop = 0;
        private PID pid;

        //Change later
        private boolean hasShotBalls = true;
        private boolean hasStartedSecondPath = false;
        private boolean turned180 = false;
        private boolean firstTurnPass = true;
        private boolean shouldTurn = false;
        private boolean stage3 = false;

        private int turnLoopCount = 0;

        // Command to run to allign the robot
        private Command alignCommand;
        private Command turnAroundCommand;

        /**
         * Called in the autonomous init function to setup the required parts of the routine
         */
        public void setup(){

            //Create new auto commands to assist the pathing
           // alignCommand = new AlignShoot(alignment, shooter, ballSystem, drive);
            //turnAroundCommand = new Turn180(drive);

            // Turn off tracking
            limelightTracking = false;

            // Reset all characteristics of the robot on init
            pathing.resetProperties();

            NavX.get().reset();

            // Robot is no longer aligned
            robotAligned = false;

            // Shooter is no longer running
            runningShooter = false;

            hasShotBalls = true;
            hasStartedSecondPath = false;
            turned180 = false;
            firstTurnPass = true;
            shouldTurn = false;

            turnLoopCount = 0;
            

            // Stop the indexer
            ballSystem.getIndexer().stopIndexing();

            // Stop the shooter
            shooter.stopShooter();

            // Starts the first section of the path and tells the robot to start tracking the target when complete
            pathing.runPath(PathContainer.basicEightPartOne(), () -> nextStage(()->setTracking(false)));
                       
        }
        /**
         * Called during the autonomous periodic method to allow for active control 
         */
        public void periodic(){

            //// Initial tracking statement
            if(getTrackingStatus()){
                alignShoot();
                System.out.println("Align");

            }
            else if(!getTrackingStatus() && shouldTurn && !turned180){
                turn180();
                System.out.println("Turn");
            }
            else if(!hasStartedSecondPath && !getTrackingStatus() && turned180){
                ballSystem.getIntake().extendIntake();
                ballSystem.getIntake().runFrontIntakeForward();

                
                //Get the actual yaw value
                pathing.resetProperties();
                pathing.runPath(PathContainer.turnAndPickUp(), () -> nextStage(() -> retractAndStopIntake()));
                hasStartedSecondPath = true;
            }
            else if(stage3){
                pathing.runPath(PathContainer.driveBackToStart());
            }

            subsystemUpdater();
        }

       
        /**
         * Flip the robot 180 degrees
         */
        private boolean turn180(){

            // If its the first time turning
            if(firstTurnPass){
                pid.setSetpoint(180);
                drive.enableOpenRampRate(1);
                firstTurnPass = false;
                
            }
            else{
            
                double power = pid.calcOutput(NavX.get().getAngle());

                // Check if the robots output power is less than 0.26 motor power if so apply an additional power of 0.3 ontop of the current power
                if(Math.abs(power) < 0.26){
                    power += Math.copySign(0.3, power);
                }

                if(pid.isInRange()){
                    drive.arcadeDrive(0, 0);
                    drive.enableOpenRampRate(0);
                    turned180 = true;
                    if(turnLoopCount == 5)
                        return true;
                    else
                        turnLoopCount++;
                }
                else{
                    drive.arcadeDrive(power, 0);
                    return false;
                }
            }
            return false;
        }

        /**
         * Allows the robot to align and shoot balls into the goal
         */
        private void alignShoot(){

            // Order is important so that the control loop doesn't run if the robot is already aligned
            if(!robotAligned && alignment.controlLoop()){
                if(alignLoop == 10){
                    robotAligned = true;
                    shouldTurn = true;
                    
                }
                else
                    alignLoop++;
            }

            // // Check if the robot is already running the shooter, if not start it
            else if(!runningShooter){
                shooter.enableShooter();
                System.out.println(shooter.getStatus());
                if(shooter.getStatus()){
                    shooter.stopShooter();
                    runningShooter = false;
                    
                    setTracking(false);
                }
                
            }

            // Finally check if the robot is aligned at the shooter is at "Full speed", if so start the belts
        //     else if(robotAligned == true && shooter.isFull() && !hasShotBalls){
        //         ballSystem.getIndexer().standardIndex();
        //        alignLoop = 0;
        //     }

        //     // Turn 180 if not already
        //    else if(!hasStartedSecondPath && turn180() && !turned180){
        //         turned180 = true;
        //     }

            


            // Update the shooter toggle
            
        }

        /**
         * Called in auto init and will run the entire robot path
         */
        public void runAuto(){

             // Run the turn around command after the align command is finished
             alignCommand.andThen(() -> turnAroundCommand.schedule());
            
             /**
              * 1. Turn Around
              * 2. Reset the odometry
              * 3. Extend and start the intake
              * 4. Run the turn and pick up path
              * 5. Stop intake and retract
              * 6. Drive back to the start
              * 7. Change the end method of the turn around method to the align command
              * 8. Turn around
              * 9. Align and shoot
              */
             turnAroundCommand.andThen(() -> runAndReset(
                 () -> extendAndRunIntake(),
                 () -> pathing.runPath(PathContainer.turnAndPickUp(), 
                 () -> nextStage(() -> retractAndStopIntake(
                 () -> pathing.runPath(PathContainer.driveBackToStart(), () -> runMultipleCommands(
                     () -> turnAroundCommand.andThen(() -> alignCommand.schedule()),
                     () -> turnAroundCommand.schedule()
                 ))
                 )))));
 
             // Start the entire path
             pathing.runPath(PathContainer.basicEightPartOne(), () -> nextStage(()->alignCommand.schedule()));
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
     * Stop running the front intake
     */
    private void retractAndStopIntake(){
        ballSystem.getIntake().stopFrontIntake();
        ballSystem.getIntake().retractIntake();
        stage3 = true;
    }
    /**
     * Stop running the front intake and then run another command
     */
    private void retractAndStopIntake(Runnable function){
        ballSystem.getIntake().stopFrontIntake();
        ballSystem.getIntake().retractIntake();
        function.run();
        stage3 = true;
    }
    /**
     * Extend an start running the intake
     */
    private void extendAndRunIntake(){
        ballSystem.getIntake().extendIntake();
        ballSystem.getIntake().runFrontIntakeForward();
    }
    
    /**
     * Run a set of commands after resetting the properties
     */
    private void runAndReset(Runnable... functions){
       pathing.resetProperties();
       for (Runnable runnable : functions) {
           runnable.run();
        }
    }
       
   
   /**
    * Run Multiple commands in sequence
    */
    private void runMultipleCommands(Runnable...runnables){
       for (Runnable runnable : runnables) {
           runnable.run();
       }
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

    /**
     * Ten ball auto that starts from and steals balls from the opposing trench
     */
    public class TenBall{

    }

}