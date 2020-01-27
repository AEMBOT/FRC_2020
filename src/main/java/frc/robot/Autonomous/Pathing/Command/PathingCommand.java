package frc.robot.Autonomous.Pathing.Command;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import frc.robot.Utilities.Control.RAMSETE.RamseteCommand;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotConstants;
import frc.robot.Hardware.Sensors.NavX;
import frc.robot.Subsystems.DriveTrainSystem;

public class PathingCommand{

    // Create new trajectory drive
    private TrajectoryDriveSubsystem robotDrive;

    /**
     * Construct the command with a drive train reference
     * @param drive reference to the DriveTrainSystem
     */
    public PathingCommand(DriveTrainSystem drive){
        robotDrive = new TrajectoryDriveSubsystem(drive);
    }

    /**
     * Reset the drive 
     */
    public void resetOdometry(){
        robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }


    /**
     * Returns the command to run in autonomous
     * @return
     */
    public Command getPathCommand(){

        //Voltage/Speed Constraints
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotConstants.kSVolts, RobotConstants.kvVoltMetersPerSecond, RobotConstants.kaVoltMetersPerSecondSquared), 
        RobotConstants.kDriveKinematics, RobotConstants.kMaxUsableVoltage);

        //Constraints for the trajectory to follow
        TrajectoryConfig config = new TrajectoryConfig(RobotConstants.kMaxVelocityMetersPerSecond, RobotConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(RobotConstants.kDriveKinematics).addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(

            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),

                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                    new Translation2d(1.5, 0)
                ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(Math.toRadians(80))),

            // Pass config
            config
        );

        // Create the ramsete controller command with the guide
        RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory, 
            robotDrive::getPose, 
            new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta), 
            new SimpleMotorFeedforward(RobotConstants.kSVolts, 
            RobotConstants.kvVoltMetersPerSecond,
            RobotConstants.kaVoltMetersPerSecondSquared), 
            RobotConstants.kDriveKinematics, 
            robotDrive::getWheelSpeeds, 
            new PIDController(RobotConstants.kPDriveVal, 0, 0), 
            new PIDController(RobotConstants.kPDriveVal, 0, 0),
            robotDrive::tankDriveVolts,
            robotDrive);

        // Return the command stating that the robot should halt after the path is complete
        return ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));
    }
}