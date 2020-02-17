package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotConstants;
import frc.robot.RobotMap;
import frc.robot.Communication.Dashboard.Dashboard;
import frc.robot.Utilities.Control.SmartMotion;

/**
 * Implementation of an Arc Shooter
 */
public class ArcShooter{

    private CANSparkMax flywheelMotor;
    private CANSparkMax flywheelMotor2;

    // Weather or not the shooter should be running
    private boolean toggledStatus = false;

    // If the fly wheel is up to speed
    private boolean flywheelStatus = false;

    private SmartMotion flyWheelProfile;

    //PIDF, TODO: Tune Values
    private final double[] pidf = {1.0, 0.0, 0.0, 0.0};

    //Only setup the pid controller once from inside the rpmControl method
    private boolean hasSetupPIDController = false;

    // The power to give to the flywheel motor
    private double currentFlyWheelPower = 0.0;

    public ArcShooter(){
        
        // Instance of the shooter motor
        flywheelMotor = new CANSparkMax(RobotMap.ShooterFlyWheelMotor, MotorType.kBrushless);

        flywheelMotor2 = new CANSparkMax(RobotMap.ShooterFlyWheelMotor2, MotorType.kBrushless);

        flywheelMotor2.follow(flywheelMotor);

        //Set a 300ms ramp rate for the motor
        flywheelMotor.setOpenLoopRampRate(4);

        flywheelMotor.setInverted(true);

     

    }

    /**
     * Constantly called only executed when toggled, allows for the flywheel to ramp up or down to allow for smooth-ish curve
     */
    public void runShooter(){

        //If the motor should be ramping up and the speed is less than one keep increasing
        if(toggledStatus){
           currentFlyWheelPower = 1;
        }

        //If not toggled and the motor is at full power ramp it down, quicker than speeding up
        else{
            currentFlyWheelPower = 0;
        }
        
        updateShooterStats();

        //Set the flywheel speed to the ramped speed
        flywheelMotor.set(currentFlyWheelPower);
    }

    /**
     * Returns the output current of the shooter motors as an array so it can be put onto the dashboard
     */
    public double[] getMotorCurrent(){
        double [] shooterCurrent = new double[1];
        shooterCurrent[0] = flywheelMotor.getOutputCurrent();

        return shooterCurrent;
    }

    /**
     * Run the shooter motor given a manual power
     */
    public void manualShooter(double leftPower, double rightPower){
        if(leftPower>0.1){
            flywheelMotor.set(leftPower);
        }
        else if(rightPower>0.1)
            flywheelMotor.set(-rightPower);
        else
            flywheelMotor.set(0);
        

        updateShooterStats();
    }

    /**
     * Set the RPM value to reach, using a motion profiling style
     * @param rpm the value to reach
     */
    public void rpmControl(double rpm){

        if(!hasSetupPIDController){
            // Spark Max motion profile to control a velocity motion profiling loop
            flyWheelProfile = new SmartMotion(pidf[0], pidf[1], pidf[2], pidf[3], 20.0, flywheelMotor);
            flyWheelProfile.setAcceptableRange(50);
            hasSetupPIDController = true;
        }

        flyWheelProfile.runVelocityProfile(rpm);
        
    }

    /**
     * Flip the value of the shooting variable
     */
    public void toggleShooter(){
        toggledStatus = !toggledStatus;
    }

    /**
     * Update the dashboard stats of the motor
     */
    private void updateShooterStats(){
        Dashboard.setTable("Subsystems");
        // Only update on real robot to avoid crashing the simulation
        if(RobotBase.isReal()){
            
            //Add the RPM values to the smart dashboard
            Dashboard.setValue("Fly-Wheel-RPM", flywheelMotor.getEncoder().getVelocity()*2);
            
            //Inform the user of wheater or not the motor is up to speed
            if(flywheelMotor.getEncoder().getVelocity() > 10_000){
                Dashboard.setValue("Fly-Wheel-Speed-Status", true);
            }
            else{
                Dashboard.setValue("Fly-Wheel-Speed-Status", false);
            }

            //Adds a graph of the motor current draw
            Dashboard.setValue("Shooter-Current-Draw", getMotorCurrent());
        }
    }
}