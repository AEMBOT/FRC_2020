package frc.robot.Autonomous.Pathing.Iterative;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;

/**
 * Allows reading of CSV files to convert into paths
 * 
 * @author Will Richards
 */
public class PathParser {

    //Lists for the 3 columns of the file
    private static ArrayList<Double> xValues = new ArrayList<>();
    private static ArrayList<Double> yValues = new ArrayList<>();
    private static ArrayList<Double> rotation = new ArrayList<>();

    //The waypoints to follow
    private static ArrayList<Translation2d> waypoints = new ArrayList<>();

    //The rotations at those points
    private static ArrayList<Rotation2d> rotations = new ArrayList<>();

    //Values for the inital pose and the ending pose
    private static Pose2d initalPose = null;
    private static Pose2d endPose = null;



    /**
     * Populate the ArrayLists with the values from the CSV file
     * 
     * @param path the path to the file
     */
    private static void populateValues(String path) {

        //Init to null so compiler doesnt die
        Scanner scan = null;
        try { scan = new Scanner(new File(path)); } catch (FileNotFoundException e) {}

        //For every line in the file
        while(scan.hasNextLine()){
            String[] splitLine = scan.nextLine().split(",");

            //Add X and Y values
            xValues.add(Double.parseDouble(splitLine[0]));
            yValues.add(Double.parseDouble(splitLine[1]));
            rotation.add(Double.parseDouble(splitLine[2]));

        }
    }

    /**
     * Creates and returns an ArrayList of waypoints
     * @param path the path to find the waypoints 
     * @return a list of waypoints
     */
    private static void getWaypoints(String path){

        //Populate the X and Y lists with values
        populateValues(path);

        //Loop through all the values in the x, y and rotation arrays
        for(int i=0; i < xValues.size(); i++){

            //Sets the first waypoint to the inital pose
            if(i == 0){
                initalPose = new Pose2d(xValues.get(i), yValues.get(i), new Rotation2d(rotation.get(i)));
            }

            //Sets the last waypoint to the end pose
            else if(i == xValues.size()-1){
                endPose = new Pose2d(xValues.get(i), yValues.get(i), new Rotation2d(rotation.get(i)));
            }

            //Set everything else to an interiorPose
            else{
                waypoints.add(new Translation2d(xValues.get(i), yValues.get(i)));
                rotations.add(new Rotation2d(rotation.get(i)));
            }
        }
    }

    /**
     * Create a path passed on the loaded information
     * @return
     */
    public static Path generatePath(TrajectoryConfig config, String filePath){

        //Compute lists of waypoints and rotaions
        getWaypoints(filePath);

        //Create a new path and pass the values to it
        return new Path(config, initalPose, waypoints, endPose);
    }




}