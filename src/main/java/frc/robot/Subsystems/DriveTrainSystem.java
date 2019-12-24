package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

    /**
     * Construct the class and init all the speed controller groups
     * @param gamepad reference to the primary gamepad
     */
    public DriveTrainSystem(){
        leftSide = new SpeedControllerGroup(new CANSparkMax(RobotMap.LeftFrontMotor, MotorType.kBrushless), 
                                            new CANSparkMax(RobotMap.LeftMiddleMotor, MotorType.kBrushless), 
                                            new CANSparkMax(RobotMap.LeftBackMotor, MotorType.kBrushless));
        
        rightSide = new SpeedControllerGroup(new CANSparkMax(RobotMap.RightFrontMotor, MotorType.kBrushless), 
                                             new CANSparkMax(RobotMap.RightMiddleMotor, MotorType.kBrushless), 
                                             new CANSparkMax(RobotMap.RightBackMotor, MotorType.kBrushless));
        
        //Flip the right side motors to account for the reversed direction of the motors
        rightSide.setInverted(true);

        diffDrive = new DifferentialDrive(leftSide, rightSide);

        //Create the encoders 
        leftSideEncoder = new Encoder(RobotMap.LeftSideEncoderA, RobotMap.LeftSideEncoderB);
        rightSideEncoder = new Encoder(RobotMap.RightSideEncoderA, RobotMap.RightSideEncoderB);
    }

    /**
     * Wrapper for the differential drive arcade drive
     */
    public void arcadeDrive(double drivePower, double turnPower){
        diffDrive.arcadeDrive(drivePower, turnPower);
    }

    /**
     * Wrapper for the tank drive method in the diff drive class
     */
    public void tankDrive(double leftPower, double rightPower){
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
     * Reset both sides encoders
     */
    public void resetEncoders(){
        rightSideEncoder.reset();
        leftSideEncoder.reset();
    }

}