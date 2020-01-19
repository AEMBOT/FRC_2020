package frc.robot.Autonomous.Pathing.Iterative;


import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

/**
 * A class for easily generating paths
 * 
 * @author Will Richards
 */
public class Path{

    //List of waypoints to follow after starting
    private ArrayList<Translation2d> interiorWaypoints;

    //Variables to store the start and end poses 
    private Pose2d initialPose;
    private Pose2d endPose;

    private TrajectoryConfig config;
    
    /**
     * constructor initializes the initial pose to 0 and the end pose to 0
     * @param config the config to use in this path
     */
    public Path(TrajectoryConfig config, Pose2d endPose){

        this.config = config;

        initialPose = new Pose2d(0, 0, new Rotation2d(0));

        this.endPose = endPose;

        interiorWaypoints = new ArrayList<>();
    }
    
    /**
     * Overloaded constructor allows custom start and end poses
     * @param initialPose the wanted start pose
     * @param endPose the wanted end pose
     */
    public Path(TrajectoryConfig config, Pose2d initialPose, Pose2d endPose){

        this.config = config;

        this.initialPose = initialPose;

        this.endPose = endPose;

        interiorWaypoints = new ArrayList<>();
    }

    /**
     * Overloaded constructor that allows entirely custom values
     * @param initialPose the wanted start pose
     * @param endPose the wanted end pose
     */
    public Path(TrajectoryConfig config, Pose2d initialPose, ArrayList<Translation2d> interiorWaypoints, Pose2d endPose){

        this.config = config;

        this.initialPose = initialPose;

        this.endPose = endPose;

        this.interiorWaypoints = interiorWaypoints;
    }

    /**
     * Compute and return the trajectory
     */
    public Trajectory getTrajectory(){
        return TrajectoryGenerator.generateTrajectory(initialPose, interiorWaypoints, endPose, config);
    }

    /**
     * Adds a waypoint to pass through when creating the s-curve, kind of obsolete due to PathParser
     * @param x horizontal position of the waypoint
     * @param y vertical position of the waypoint
     */
    public void addWaypoint(double x, double y){
        interiorWaypoints.add(new Translation2d(x, y));
    }
}