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
     * Get the example straight 1 meter path
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


}