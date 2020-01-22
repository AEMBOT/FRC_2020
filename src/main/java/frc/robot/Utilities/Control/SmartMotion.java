package frc.robot.Utilities.Control;

import java.util.ArrayList;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * Wrapper class for the Spark MAX Smart Motion / Motion Profiling, Note: Only
 * input speed controller groups if the spark maxes are in them
 * 
 * @author Will Richards
 */
public class SmartMotion {

    // List of motors
    ArrayList<CANSparkMax> motorList;

    // List of PID Controllers for motors
    ArrayList<CANPIDController> pidControllerList;

    /**
     * Preset PID Coefficients kMaxOutput - The maximum power alloted to the motors
     * kMinOutput - The minimum amount of power the motor can use maxRPM - The
     * Maximum RPM of the motor (currently set to slightly higher than the free spin
     * RPM for a neo)
     */
    private double kP, kI, kD, kIz, kFF, kMaxOutput = 1, kMinOutput = -1, maxRPM = 5700, maxVel = 2000, minVel = 0,
            maxAcc = 1500, acceptableErr;

    // The slotID signifies which gain values are being used, it can be between 0-3
    // each having different gains
    private final int slotID = 0;

    /**
     * Simply takes two speed controller groups assuming both have the desired Spark
     * Maxes in them
     * 
     * @param kP P gain
     * @param kI I gain 
     * @param kD D gain
     * @param kFF Feed forward value
     * @param minVel the minimum velocity of the motor
     * @param motors array of motors the current smartmotion profiling applies to
     */
    public SmartMotion(double kP, double kI, double kD, double kFF, double minVel, CANSparkMax[] motors) {

        /**
         * PID Coefficients kP - Error kI - Error Over Time kD - Predictive Future
         * Trends kFF - Reduces The Error Faster / Keeps error smaller maxVel - Maximum
         * velocity the motor is allowed to "move" minVel - Minimum velocity the motor
         * is allowed to "move" maxAcc - Maximum acceleration the motor can preform
         */
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
        this.minVel = minVel;


        // Init all lists
        motorList = new ArrayList<>();
        pidControllerList = new ArrayList<>();

        // Adds motor specific things for each motor into lists to be used later
        for (CANSparkMax motor : motors) {
            motorList.add(motor);

            pidControllerList.add(setupPIDController(motor.getPIDController()));
        }
    }

    /**
     * Overloaded constructor that allows passing of every single necessary constant
     * modification, this one probably wont get used
     * 
     * @param kP P gain
     * @param kI I gain 
     * @param kD D gain
     * @param kFF Feed forward value
     * @param kMaxOutput the maximum motor output
     * @param kMinOutput the minimum motor output
     * @param maxVel the max velocity the motors can spin
     * @param minVel the min velocity the motors can spin
     * @param maxAcc the maximum acceleration
     * @param motors the list of motors the profile affects
     * 
     */
    public SmartMotion(double kP, double kI, double kD, double kFF, double kMaxOutput, double kMinOutput, double maxVel,
            double minVel, double maxAcc, CANSparkMax[] motors) {

        /**
         * PID Coefficients kP - Error kI - Error Over Time kD - Predictive Future
         * Trends kFF - Reduces The Error Faster / Keeps error smaller maxVel - Maximum
         * velocity the motor is allowed to "move" minVel - Minimum velocity the motor
         * is allowed to "move" maxAcc - Maximum acceleration the motor can preform
         * kMaxOutput - Maximum power output kMinOutput - Minimum power output
         */
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
        this.maxVel = maxVel;
        this.minVel = minVel;
        this.maxAcc = maxAcc;
        this.kMaxOutput = kMaxOutput;
        this.kMinOutput = kMinOutput;

        // Init all lists
        motorList = new ArrayList<>();
        pidControllerList = new ArrayList<>();

        // Adds motor specific things for each motor into lists to be used later
        for (CANSparkMax motor : motors) {
            motorList.add(motor);

            pidControllerList.add(setupPIDController(motor.getPIDController()));
        }
    }

    /**
     * Applies all constants to the controller and returns a reference to it
     * 
     * @param controller the controller to apply parameters to
     * @return the modified controller
     */
    private CANPIDController setupPIDController(CANPIDController controller) {

        // Setup PIDF constants and ranges
        controller.setP(this.kP);
        controller.setI(this.kI);
        controller.setD(this.kD);
        controller.setIZone(this.kIz);
        controller.setFF(this.kFF);
        controller.setOutputRange(this.kMinOutput, this.kMaxOutput);

        /**
         * Setup motor velocity and motion variables setSmartMotionMaxVelocity - The max
         * velocity the motor will move and where the profile will spend the most amount
         * of time setSmartMotionMinOutputVelocity - The minimum velocity the motor can
         * be at, any values below this will be zeroed setSmartMotionMaxAccel - The rate
         * at which the velocity will increase until the max velocity is reached
         * setSmartMotionAllowedClosedLoopError - The acceptable amount of error in the
         * system
         */
        controller.setSmartMotionMaxVelocity(this.maxVel, slotID);
        controller.setSmartMotionMinOutputVelocity(this.minVel, slotID);
        controller.setSmartMotionMaxAccel(this.maxAcc, slotID);
        controller.setSmartMotionAllowedClosedLoopError(this.acceptableErr, slotID);


        return controller;
    }

    /**
     * Sets the range at which point the controller will stop trying to proceed
     * 
     * @param acceptableErr the value to use as the error
     */
    public void setAcceptableRange(double acceptableErr) {
        this.acceptableErr = acceptableErr;
    }

    /**
     * Attempts to reach and maintain a set velocity
     * 
     * @param setpoint the velocity we are trying to reach (in RPM)
     */
    public void runVelocityProfile(double setpoint) {
        for (CANPIDController controller : pidControllerList) {
            controller.setReference(setpoint, ControlType.kSmartVelocity);
        }
    }

    /**
     * Attempts to "smoothly" drive a distance utilizing the SmartMotion and built
     * in PID
     * 
     * @param setpoint the point we want to reach (in rotations)
     */
    public void runPositionProfile(double setpoint) {
        for (CANPIDController controller : pidControllerList) {
            controller.setReference(setpoint, ControlType.kSmartMotion);
        }
    }

}