package frc.robot.Autonomous.Control;

import frc.robot.RobotConstants;
import frc.robot.Hardware.Sensors.NavX;
import frc.robot.Subsystems.DriveTrainSystem;
import frc.robot.Utilities.Control.PID;

/**
 * Class created to handle basic bare bones autonomous driving, mainly for use if pathing is unsuccessful
 * 
 * @author Will Richards
 */
public class AutoDriveControl {

    // Get a local instance of the Drive Train System
    private DriveTrainSystem drive;

    // Holds the PID object which allows for precise movements, for turing and
    // driving respectively
    private PID turnPID;
    private PID drivePID;

    // Instance of the NavX
    private NavX navX;

    // PID Constants for turning, TODO: Tune
    private final double turn_kP = 0.001;
    private final double turn_kI = 0;
    private final double turn_kD = 0;

    // PID constants for driving, TODO: Tune
    private final double drive_kP = 1;
    private final double drive_kI = 0;
    private final double drive_kD = 0;

    /**
     * Create a constructor to pass the drive train sides into
     */
    public AutoDriveControl(DriveTrainSystem driveTrainSystem) {

        // Creates local instances of the drive train
        this.drive = driveTrainSystem;

        // Gets the static reference to the NavX
        navX = NavX.get();

        // Creates a new PID controller that will account for the overflow of the NavX
        // when turning
        turnPID = new PID(turn_kP, turn_kI, turn_kD);
        turnPID.setAcceptableRange(1);

        // Creates a new PID controller to handle accurate of distances, larger range
        // because working in ticks instead of degrees
        drivePID = new PID(drive_kP, drive_kI, drive_kD);
        drivePID.setAcceptableRange(0.02);
    }

    /**
     * Method created to handle simple distance driving, returns bool to signify
     * when complete
     * 
     * @param distance the wanted distance in inches
     * @return the status of completion
     */
    public boolean DriveDistance(double distance) {

        // Set the point for the PID loop that we want to reach
        drivePID.setSetpoint(distance);

        // Calculate the value needed to reach that point
        double motorPower = drivePID.calcOutput(drive.getAverageEncoderDistance());

        System.out.println("Setpoint: " + distance);
        System.out.println("Position: " + drive.getAverageEncoderDistance());


        if(Math.abs(motorPower) > 0.5){
            motorPower = Math.copySign(0.5, motorPower);
        }

        // If in range stop the robot and report that the loop is done
        if (drivePID.isInRange()) {
            drive.arcadeDrive(0, 0);
            drive.resetEncoders();
            return true;
        }

        // If not in range keep driving and tell the program that it hasn't finished its
        // loop yet
        else {
            drive.arcadeDrive(motorPower, 0);
            return false;
        }

    }

    /**
     * Method used for basic turning to face an angle, angles are in degrees
     * 
     * @param angle the angle we want to reach
     * @return the status of its completion
     */
    public boolean TurnToAngle(double angle) {

        // Use the yaw corrected from 0-180 to 0-360 and pass it as the input to the PID
        // loop
        double power = turnPID.calcOutput(navX.getEdgeCaseAngle(angle));
        System.out.println("Calculated Angle: " + navX.getEdgeCaseAngle(angle));

        if(Math.abs(power) > 0.5){
            power = Math.copySign(0.5, power);
        }

        // Check if the robot is in range yet and if so stop and return true saying the
        // manuever is complete
        if (turnPID.isInRange()) {
            drive.arcadeDrive(0, 0);
            drive.resetEncoders();
            return true;
        }

        // If not in range keep driving and return false saying it is incomplete
        else {
            drive.arcadeDrive(0, power);
            return false;
        }
    }

}