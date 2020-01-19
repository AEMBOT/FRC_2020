package frc.robot.Autonomous.Pathing;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.RobotConstants;
import frc.robot.Hardware.Sensors.NavX;
import frc.robot.Subsystems.DriveTrainSystem;

/**
 * Class derived from the WPILIB Trajectory Drive (https://docs.wpilib.org/en/latest/docs/software/trajectory-end-to-end/creating-drive-subsystem.html)
 * This program essentially is just a more specific variant of robot drive train control
 * 
 * @author Will Richards
 */
public class TrajectoryDrive{

    // Reference to the drive train
    private DriveTrainSystem drive;

    // Local references of the Encoders
    private Encoder leftDriveTrainEncoder;
    private Encoder rightDriveTrainEncoder;

    // Local reference to the NavX
    private NavX navX;

    // Keeps track of the robots position on the field
    private final DifferentialDriveOdometry odometry;
    

    /**
     * TrajectoryFollow constructor
     * @param drive reference to a drive train class
     */
    public TrajectoryDrive(DriveTrainSystem drive){
        this.drive = drive;

        // Get instances of the dt encoders
        leftDriveTrainEncoder = drive.getLeftSideEncoder();
        rightDriveTrainEncoder = drive.getRightSideEncoder();

        // Get a reference to the NavX
        navX = NavX.get();

        //Initialize the robot odometry
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getSpecificHeading()));
        
    }

    /**
     * Drive the robot with a tank style drive, but pass voltages to the motor controllers instead of "powers"
     */
    public void tankDriveVolts(double leftVolts, double rightVolts){
        drive.getLeftSideMotors().setVoltage(leftVolts);
        drive.getRightSideMotors().setVoltage(rightVolts);

        System.out.println("Trans: " + getPose().getTranslation());
        System.out.println("Rot: " + getPose().getRotation());
    }

    /**
     * Used to update the estimated position of the robot
     */
    public void updateOdometry(){
        odometry.update(Rotation2d.fromDegrees(getSpecificHeading()), leftDriveTrainEncoder.getDistance(), rightDriveTrainEncoder.getDistance());
    }

    /**
     * Returns the currently estimated pose of the robot
     * @return the pose
     */
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    /**
     * Gets the currents speeds of each output shaft
     * @return the current wheel speeds
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(leftDriveTrainEncoder.getRate(), rightDriveTrainEncoder.getRate());
    }

    /**
     * Reset the odometry to a know pose
     * @param pose the pose we wish to reset to
     */
    public void resetOdometry(Pose2d pose){
        drive.resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getSpecificHeading()));
    }

    /**
     * Gets modified heading specifically for trajectory planning
     * @return the altered heading
     */
    public double getSpecificHeading(){
        return Math.IEEEremainder(navX.getAngle(), 360) * (RobotConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate factoring in weather or not the gyro is reversed
     */
    public double getTurnRate(){
        return navX.getRate() * (RobotConstants.kGyroReversed ? -1.0 : 1.0);
    }
    
}