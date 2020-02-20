package frc.robot.Autonomous.Pathing;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.Hardware.Sensors.NavX;
import frc.robot.Subsystems.DriveTrainSystem;
import frc.robot.Utilities.Control.RAMSETE.RamseteCommand;

public class PathingCommand{

    // Create new trajectory drive
    private TrajectoryDriveSubsystem robotDrive;

    RamseteCommand ramseteCommand;

    /**
     * Construct the command with a drive train reference
     * @param drive reference to the DriveTrainSystem
     */
    public PathingCommand(DriveTrainSystem drive){
        robotDrive = new TrajectoryDriveSubsystem(drive);
    }

    /**
     * Reset the artifical position of the robot
     */
    public void resetOdometry(){
        robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

    /**
     * Sets the position / rotation of the robot to a set position based on a given value
     */
    public void setOdometry(double x, double y, double angle){
        robotDrive.resetOdometry(new Pose2d(x, y, new Rotation2d(angle)));
    }

    /**
     * Reset all important parts of the path, NavX, Encoders, Odometry, etc.
     */
    public void resetProperties(){
        robotDrive.resetEncoders();
        NavX.get().resetYaw();
        resetOdometry();
    }

     /**
     * Returns the command to run in autonomous
     * @return
     */
    public Command getPathCommand(Path path, boolean inverted){


    robotDrive.setInverted(inverted);
    //Voltage/Speed Constraints
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotConstants.kSVolts, RobotConstants.kvVoltMetersPerSecond, RobotConstants.kaVoltMetersPerSecondSquared), 
    RobotConstants.kDriveKinematics, RobotConstants.kMaxUsableVoltage);

    //Constraints for the trajectory to follow
    TrajectoryConfig config = new TrajectoryConfig(RobotConstants.kMaxVelocityMetersPerSecond, RobotConstants.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(RobotConstants.kDriveKinematics).addConstraint(autoVoltageConstraint);

       // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = path.getTrajectory();

    if(!inverted){
        // Create the ramsete controller command with the guide
        ramseteCommand = new RamseteCommand(
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
    }
    else{
        // Create the ramsete controller command with the guide
        ramseteCommand = new RamseteCommand(
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
            robotDrive::inverseTankDriveVolts,
            robotDrive);
    }

        // Return the command stating that the robot should halt after the path is complete
        return ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));
    }

     /**
     * Returns the command to run in autonomous, as well as giving the ability to run something after the path is complete
     * @return
     */
    public Command getPathCommand(Path path, boolean inverted, Runnable endAction){

    robotDrive.setInverted(inverted);
        //Voltage/Speed Constraints
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotConstants.kSVolts, RobotConstants.kvVoltMetersPerSecond, RobotConstants.kaVoltMetersPerSecondSquared), 
        RobotConstants.kDriveKinematics, RobotConstants.kMaxUsableVoltage);
    
        //Constraints for the trajectory to follow
        TrajectoryConfig config = new TrajectoryConfig(RobotConstants.kMaxVelocityMetersPerSecond, RobotConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(RobotConstants.kDriveKinematics).addConstraint(autoVoltageConstraint);
    
           // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = path.getTrajectory();
    
        if(!inverted){
            // Create the ramsete controller command with the guide
            ramseteCommand = new RamseteCommand(
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
        }
        else{
            // Create the ramsete controller command with the guide
            ramseteCommand = new RamseteCommand(
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
                robotDrive::inverseTankDriveVolts,
                robotDrive);
        }
    
            // Return the command stating that the robot should halt after the path is complete
            return ramseteCommand.andThen(endAction);
        }
}