package frc.robot.Utilities.Control;

import frc.robot.Hardware.Vision.Limelight;
import frc.robot.Subsystems.DriveTrainSystem;

public class LimelightAlignment{

    public Limelight limelight;

    private PID pid;
    private DriveTrainSystem drive;
    private boolean initalControlLoop = true;

    // Limelight values, the current X offset of the angle
    private double limelightX = 0;

    // The range around the setpoint that is considered in range (degrees)
    private double approxRange = 1.9;
    
    // The power to be given to the motors from the PID loop
    private double power = 0;

    // Constant
    private final double staticFricitionOffset = 0.2;


    /**
     * The constructor for aligning the robot to the target
     */
    public LimelightAlignment(DriveTrainSystem drive){
        limelight = new Limelight();

        // P, I, D, staticFrictionOffset
        pid = new PID(.029,0.0,0.0, 0.2);
        pid.setAcceptableRange(approxRange);
        
        this.drive = drive;
    }

    /**
     * Run to align the robot
     */
    public void controlLoop(){
        limelightX = limelight.getX(); 
        approxRange = 1.9; 
        power = 0;
        power = pid.calcOutput(limelightX);

        // Check if the PID value is in range and there is more than one active target
        if(pid.isInRange() && limelight.getValidTarget() > 0){
            System.out.println("Aligned");
            drive.arcadeDrive(0, 0);
        }

        // If not aligned then attempt to align 
        else{
            drive.arcadeDrive(power, 0);
        }
    }
}