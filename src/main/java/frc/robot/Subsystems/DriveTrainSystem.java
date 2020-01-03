package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotConstants;
import frc.robot.RobotMap;

public class DriveTrainSystem {

    //Drive train speed controller variables
    private SpeedControllerGroup leftSide;
    private SpeedControllerGroup rightSide;


    //Local instance of the Differential Drive class
    private DifferentialDrive diffDrive;

    //Create variables for the through bore encoders on either side of the drive train
    private Encoder leftSideEncoder;
    private Encoder rightSideEncoder;

    //Implemented early to prevent issues regarding slowing down the drive train for demos
    private boolean enableDemoMode = false;

    //As well set a power cap while in demo mode
    private double maximumDemoPower = 0.45;

    //Individual Left Side Motors
    private CANSparkMax LeftFrontMotor;
    private CANSparkMax LeftMiddleMotor;
    private CANSparkMax LeftBackMotor;

    //Individual Right Side Motors
    private CANSparkMax RightFrontMotor;
    private CANSparkMax RightMiddleMotor;
    private CANSparkMax RightBackMotor;

    /**
     * Construct the class and init all the speed controller groups
     * @param gamepad reference to the primary gamepad
     */
    public DriveTrainSystem(){

        //Create the individual motors for the left side to add to the SpeedControllerGroup
        LeftFrontMotor = new CANSparkMax(RobotMap.LeftFrontMotor, MotorType.kBrushless);
        LeftMiddleMotor =  new CANSparkMax(RobotMap.LeftMiddleMotor, MotorType.kBrushless);
        LeftBackMotor = new CANSparkMax(RobotMap.LeftBackMotor, MotorType.kBrushless);

        //Create the individual motors for the right side to add to the SpeedControllerGroup
        RightFrontMotor = new CANSparkMax(RobotMap.RightFrontMotor, MotorType.kBrushless);
        RightMiddleMotor =  new CANSparkMax(RobotMap.RightMiddleMotor, MotorType.kBrushless);
        RightBackMotor = new CANSparkMax(RobotMap.RightBackMotor, MotorType.kBrushless);

        //SpeedControllerGroups that hold all meaningful 
        leftSide = new SpeedControllerGroup(LeftFrontMotor, LeftMiddleMotor, LeftBackMotor);
        rightSide = new SpeedControllerGroup(RightFrontMotor, RightMiddleMotor, RightBackMotor);
        
        
        //Flip the right side motors to account for the reversed direction of the motors
        rightSide.setInverted(true);

        diffDrive = new DifferentialDrive(leftSide, rightSide);

        //Create the encoders 
        leftSideEncoder = new Encoder(RobotMap.LeftSideEncoderA, RobotMap.LeftSideEncoderB);
        rightSideEncoder = new Encoder(RobotMap.RightSideEncoderA, RobotMap.RightSideEncoderB);

        // Convert the pulses into usable distances
        leftSideEncoder.setDistancePerPulse(RobotConstants.kEncoderDistancePerPulse);
        rightSideEncoder.setDistancePerPulse(RobotConstants.kEncoderDistancePerPulse);
    }

    /**
     * Wrapper for the differential drive arcade drive
     */
    public void arcadeDrive(double drivePower, double turnPower){

        //Cap power to max if in demo mode
        if(enableDemoMode)
            if(drivePower > maximumDemoPower)
                drivePower = maximumDemoPower;
        
        diffDrive.arcadeDrive(drivePower, turnPower);
    }

    /**
     * Drive the robot with a tank style drive, but pass voltages to the motor controllers instead of "powers"
     */
    public void tankDriveVolts(double leftVolts, double rightVolts){
        getLeftSideMotors().setVoltage(leftVolts);
        getRightSideMotors().setVoltage(rightVolts);
    }

    /**
     * Wrapper for the tank drive method in the diff drive class
     */
    public void tankDrive(double leftPower, double rightPower){

        //Cap power to max if in demo mode
        if(enableDemoMode){
            if(leftPower > maximumDemoPower)
                leftPower = maximumDemoPower;
            if(rightPower > maximumDemoPower)
                rightPower = maximumDemoPower;
        }

        diffDrive.tankDrive(leftPower, rightPower);
    }

    
    /**
     * Get the left-side's speed controller group
     * @return leftSide
     */
    public SpeedControllerGroup getLeftSide(){
        return leftSide;
    }

    /**
     * Get the right-side's speed controller group
     * @return rightSide
     */
    public SpeedControllerGroup getRightSide(){
        return rightSide;
    }

    /**
     * Get the value from the left side encoder
     */
    public int getLeftSideEncoderPosition(){
        return leftSideEncoder.get();
    }

    /**
     * Get a reference to the encoder on the left side
     * @return leftSideEncoder
     */
    public Encoder getLeftSideEncoder(){
        return leftSideEncoder;
    }

     /**
     * Get a reference to the encoder on the right side
     * @return rightSideEncoder
     */
    public Encoder getRightSideEncoder(){
        return rightSideEncoder;
    }

    /**
     * Get the value from the right side encoder
     */
    public int getRightSideEncoderPosition(){
        return rightSideEncoder.get();
    }

    /**
     * Gets the average position between the two sides 
     * @return the averaged position
     */
    public int getAveragePosition(){
        return ((getLeftSideEncoderPosition() + getRightSideEncoderPosition()) / 2);
    }

    /**
     * The average distance traveled between both encoders
     * @return the distance
     */
    public double getAverageEncoderDistance(){
        return ((leftSideEncoder.getDistance() + rightSideEncoder.getDistance()) / 2.0);
    }

    /**
     * Reset both sides encoders
     */
    public void resetEncoders(){
        rightSideEncoder.reset();
        leftSideEncoder.reset();
    }

    /**
     * Gets an array of the motors on the left side
     * @return array of Spark Maxes
     */
    public SpeedControllerGroup getLeftSideMotors(){
        return leftSide;
    }

    /**
     * Gets an array of the motors on the right side
     * @return array of Spark Maxes
     */
    public SpeedControllerGroup getRightSideMotors(){
        return rightSide;
    }

}