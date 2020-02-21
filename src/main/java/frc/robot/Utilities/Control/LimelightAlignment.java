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


    /**
     * The constructor for aligning the robot to the target
     */
    public LimelightAlignment(DriveTrainSystem drive){
        limelight = new Limelight();

        // P, I, D, staticFrictionOffset
        pid = new PID(.027,0,0.0);
        pid.setAcceptableRange(approxRange);
       // pid.setLoopRequirement(10);
        
        
        this.drive = drive;
        
    }

    /**
     * Run to align the robot
     */
    public boolean controlLoop(){
        if(initalControlLoop){
            this.drive.enableClosedRampRate(0.05);
            initalControlLoop = false;
        }
        limelightX = limelight.getX(); 
        power = 0;
        power = pid.calcOutput(limelightX*-1);

        // Check if the robots output power is less than 0.26 motor power if so apply an additional power of 0.3 ontop of the current power
        if(Math.abs(power) < 0.26){
            power += Math.copySign(0.3, power);
        }

        // Check if the PID value is in range and there is more than one active target
        if(pid.isInRange() && limelight.getValidTarget() > 0){
            //System.out.println("Aligned");
            drive.arcadeDrive(0, 0);
            return true;
        }

        // If not aligned then attempt to align 
        else{
            drive.arcadeDrive(power, 0);
            return false;
        }
    }
}