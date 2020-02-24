package frc.robot.Autonomous.Pathing;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.RobotConstants;

/**
 * Container created to hold large numbers of auto path, and methods to return them
 * 
 * @author Will Richards
 */
public class PathContainer{

    /**
     * Setup the actual drive train configuration variable
     */
    private static TrajectoryConfig getConfig(){
        
        //Voltage/Speed Constraints
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotConstants.kSVolts, RobotConstants.kvVoltMetersPerSecond, RobotConstants.kaVoltMetersPerSecondSquared), 
        RobotConstants.kDriveKinematics, RobotConstants.kMaxUsableVoltage);

        //Constraints for the trajectory to follow
        TrajectoryConfig Localconfig = new TrajectoryConfig(RobotConstants.kMaxVelocityMetersPerSecond, RobotConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(RobotConstants.kDriveKinematics).addConstraint(autoVoltageConstraint);

        return Localconfig;
    }

      /**
     * Setup the actual drive train configuration variable
     */
    private static TrajectoryConfig getConfig(double maxVelocity){
        
        //Voltage/Speed Constraints
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotConstants.kSVolts, RobotConstants.kvVoltMetersPerSecond, RobotConstants.kaVoltMetersPerSecondSquared), 
        RobotConstants.kDriveKinematics, RobotConstants.kMaxUsableVoltage);

        //Constraints for the trajectory to follow
        TrajectoryConfig Localconfig = new TrajectoryConfig(maxVelocity, RobotConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(RobotConstants.kDriveKinematics).addConstraint(autoVoltageConstraint);

        return Localconfig;
    }

    /**
     * Get the example path of driving 3 meters with a curve at 1 meter and coming back in line at 2 meters
     * 
     * @return the path
     */
    public static Path getExamplePath(){
        
        // Trajectory Config, End Pose at 90 degrees (80 cause weird over shoot)
        Path path = new Path(getConfig(), new Pose2d(3, 0, new Rotation2d(0)));

        path.addWaypoint(1, 0.5);
        path.addWaypoint(2, 0);

        return path;
    }

    //region 8 Ball Auto Paths

    /**
     * Backs the robot up 1.4m and aligns
     * @return the path to complete this action
     */
    public static Path basicEightPartOne(){

        // Start at (0,0) (Subject to change) drive backwards because of the inverse "1 meter" with a waypoint .75 meters in the middle
        Path path = new Path(getConfig(), new Pose2d(1, 0, new Rotation2d(0)));

        path.setInverted(true);

        path.addWaypoint(0.75, 0);

        return path;
    }

    /**
     * Turn the robot around and pick up 3 balls
     * @return
     */
    public static Path turnAndPickUp(){
        Path path = new Path(getConfig(0.8), new Pose2d(2, 0, new Rotation2d(Math.toRadians(0))));

        path.setInverted(false);

        path.addWaypoint(1, 0);



        return path;
    }

    /**
     * Drive from picking up the balls backwards to where we started
     */
    public static Path driveBackToStart(){
        Path path = new Path(getConfig(), new Pose2d(3, 0, new Rotation2d(0)));

        path.setInverted(true);

        path.addWaypoint(2.5, 0);

        return path;
    }

    //endregion

    //region 10 Ball Auto Paths

    /**
     * Grab the balls from the opposing trench
     * 
     * Drive forward 3.5 meters to the trench with a max velocity of 0.75m/s
     * 
     * @return the path to complete this action
     */
    public static Path grabBallsFromOpposingTrench(){
        Path path = new Path(getConfig(0.75), new Pose2d(3.467, 0, new Rotation2d(0)));

        path.addWaypoint(1.7335, 0);

        path.setInverted(false);

        return path;
    }

    /**
     * Drive from the opposing trench to the center of the field to shoot all 5 balls
     * 
     * Initial Position: (3.467, 0) and 0 degrees
     * Final Position  (1.77, 0) and 0 degrees
     * 
     * Added because it thinks its going forward
     * 
     * @return the path to complete this action
     */
    public static Path backOutOfTrench(){
        Path path = new Path(getConfig(), new Pose2d(3.467, 0, new Rotation2d(0)), new Pose2d(5.237, 0, new Rotation2d(0)));

        path.setInverted(true);

        path.addWaypoint(4.352, 0);

        return path;

    }

    /**
     * After backing out of the trench we need to drive and shoot into the goal
     * 
     * Initial Position: (1.77, 0) and 0
     * Final Position: (1.014, 5.223)
     * 
     * TODO: May need to turn to make the path work
     * 
     * @return path to complete
     */
    public static Path driveToShoot(){
        Path path = new Path(getConfig(), new Pose2d(1.77, 0, new Rotation2d(0)), new Pose2d(1.014, 5.223, new Rotation2d(Math.toRadians(-180))));

        path.setInverted(false);

        path.addWaypoint(1.985, 1.30575);
        path.addWaypoint(1.77, 2.6115);
        path.addWaypoint(1.392, 5.223);

        return path;
    }


    //endregion

}