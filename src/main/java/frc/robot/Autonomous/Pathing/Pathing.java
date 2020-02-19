package frc.robot.Autonomous.Pathing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrainSystem;

/**
 * Overall pathing class used to control all the robots auto paths
 * 
 * @author Will Richards
 */
public class Pathing{

    // PathingCommand generation and command
    private PathingCommand pathingCommand;
    private Command pathCommand;

    /**
     * Constructor for running auto paths
     * @param drive
     */
    public Pathing(DriveTrainSystem drive){
        pathingCommand = new PathingCommand(drive);
    }

    /**
     * Run the selected path
     * @param path
     */
    public void runPath(Path path){
        
        // Get the actual auto path command we will be running
        pathCommand = pathingCommand.getPathCommand(path);

        if(pathCommand != null && !pathCommand.isScheduled()){
        pathCommand.schedule();
        }
    }

    /**
     * Resets all the physical location properties of the robot
     */
    public void resetProperties(){
        pathingCommand.resetProperties();
    }
}