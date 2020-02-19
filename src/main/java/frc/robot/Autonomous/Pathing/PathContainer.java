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

    private static TrajectoryConfig config;

    /**
     * Setup the actual drive train configuration variable
     */
    public static void setupConfig(){
        
        //Voltage/Speed Constraints
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotConstants.kSVolts, RobotConstants.kvVoltMetersPerSecond, RobotConstants.kaVoltMetersPerSecondSquared), 
        RobotConstants.kDriveKinematics, RobotConstants.kMaxUsableVoltage);

        //Constraints for the trajectory to follow
        TrajectoryConfig Localconfig = new TrajectoryConfig(RobotConstants.kMaxVelocityMetersPerSecond, RobotConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(RobotConstants.kDriveKinematics).addConstraint(autoVoltageConstraint);

        config = Localconfig;
    }

    /**
     * Get the example sin like path
     * 
     * @return the path
     */
    public static Path getExamplePath(){
        
        // Trajectory Config, End Pose at 90 degrees (80 cause weird over shoot)
        Path path = new Path(config, new Pose2d(3, 0, new Rotation2d(Math.toRadians(80))));

        path.addWaypoint(1.5, 0);

        return path;
    }
}