package frc.robot.Autonomous.Pathing;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotConstants;
import frc.robot.Subsystems.DriveTrainSystem;

/**
 * Generates and follows a trajectory based path
 * 
 * @author Will Richards
 */
public class TrajectoryFollow {

    private DifferentialDriveVoltageConstraint autoVoltageConstraint;
    private TrajectoryConfig pathConfig;

    //A test path to get started
    private Path testPath;

    //The true trajectory derived from the path
    private Trajectory testTrajectory;

    //The path following command
    private RamseteCommand ramseteCommand;

    //The trajectory based robot controller variable
    private TrajectoryDrive trajectoryDrive;

    private boolean hasRunCommand = false;


    /**
     * Create all important path variables (constraints, etc.)
     */
    public TrajectoryFollow(DriveTrainSystem drive){

        //Create a voltage constraint do we dont accelerate too fast
        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotConstants.kSVolts, RobotConstants.kvVoltMetersPerSecond, RobotConstants.kaVoltMetersPerSecondSquared),
        RobotConstants.kDriveKinematics, RobotConstants.kMaxUsableVoltage);

        trajectoryDrive = new TrajectoryDrive(drive);

        //Create the path configuration
        pathConfig = new TrajectoryConfig(RobotConstants.kMaxVelocityMetersPerSecond, RobotConstants.kMaxAccelerationMetersPerSecondSquared);

        //Add the drive Kinematics to the config to make sure max speed is actually obeyed
        pathConfig.setKinematics(RobotConstants.kDriveKinematics);

        //Make sure it plans for the voltage constraint
        pathConfig.addConstraint(autoVoltageConstraint);

        //Create path and add points (1,1) and (2,-1) and set the end point to 3 meters ahead
        testPath = new Path(pathConfig, new Pose2d(3, 0, new Rotation2d(0)));
        testPath.addWaypoint(1, 1);
        testPath.addWaypoint(2, -1);

        testTrajectory = testPath.getTrajectory();

    }

    /**
     * Starts following the path, call in auto init
     */
    public boolean followPath(){
        if(!hasRunCommand){
            RamseteController controller = new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta);

            SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(RobotConstants.kSVolts, RobotConstants.kvVoltMetersPerSecond, RobotConstants.kaVoltMetersPerSecondSquared);

            PIDController motorControlPID = new PIDController(RobotConstants.kPDriveVal, 0, 0);

            ramseteCommand = new RamseteCommand(testTrajectory, trajectoryDrive::getPose, controller, feedforward, RobotConstants.kDriveKinematics, trajectoryDrive::getWheelSpeeds, motorControlPID, motorControlPID, trajectoryDrive::tankDriveVolts);

            //Starts the command and then when its done tells the robot to stop moving
            ramseteCommand.andThen(() -> trajectoryDrive.tankDriveVolts(0, 0));

            hasRunCommand = true;
        }

        //Tell the iterative program that the command has finished
        if(ramseteCommand.isFinished()){
            return true;
        }
        else{
            return false;
        }
    }
}