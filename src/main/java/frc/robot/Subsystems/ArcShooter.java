package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotMap;
import frc.robot.Communication.Dashboard.Dashboard;

/**
 * Implementation of an Arc Shooter
 */
public class ArcShooter{

    // Left motor in the shooter gear box
    private CANSparkMax leftShooterMotor;

    private CANSparkMax rightShooterMotor;

    // Weather or not the shooter should be running
    private boolean toggledStatus = false;

    // If the fly wheel is up to speed
    private boolean flywheelStatus = false;

    // The power to give to the flywheel motor
    private double currentFlyWheelPower = 0.0;

    public ArcShooter(){

        // Instance of the shooter motor
        leftShooterMotor = new CANSparkMax(RobotMap.ShooterMotorLeft, MotorType.kBrushless);
        
        // Instance of the shooter motor
        rightShooterMotor = new CANSparkMax(RobotMap.ShooterMotorRight, MotorType.kBrushless);
    
        leftShooterMotor.setInverted(true);

        // Follow the left shooter motor to keep them moving at the same rate
        rightShooterMotor.follow(leftShooterMotor);

    }

    /**
     * Constantly called only executed when toggled, allows for the flywheel to ramp up or down to allow for smooth-ish curve
     */
    public void runShooter(){

        //If the motor should be ramping up and the speed is less than one keep increasing
        if(toggledStatus){
            if(currentFlyWheelPower < 1)
                currentFlyWheelPower += 0.05;
        }

        //If not toggled and the motor is at full power ramp it down, quicker than speeding up
        else{
            if(currentFlyWheelPower >= 0.1){
                currentFlyWheelPower -= 0.1;
            }
            else{
                currentFlyWheelPower = 0;
            }
        }
        
        updateShooterStats();

        //Set the flywheel speed to the ramped speed
        leftShooterMotor.set(currentFlyWheelPower);
    }

    /**
     * Returns the output current of the shooter motors as an array so it can be put onto the dashboard
     */
    public double[] getMotorCurrent(){
        double [] shooterCurrent = new double[2];
        shooterCurrent[0] = leftShooterMotor.getOutputCurrent();
        shooterCurrent[1] = rightShooterMotor.getOutputCurrent();

        return shooterCurrent;
    }

    /**
     * Run the shooter motor given a manual power
     */
    public void manualShooter(double leftPower, double rightPower){
        if(leftPower>0.1)
            leftShooterMotor.set(leftPower);
        else if(rightPower>0.1)
            leftShooterMotor.set(rightPower  * -1);
        else{
            leftShooterMotor.set(0);
        }

        updateShooterStats();
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
            Dashboard.setValue("Fly-Wheel-RPM", leftShooterMotor.getEncoder().getVelocity());
            
            //Inform the user of wheater or not the motor is up to speed
            if(leftShooterMotor.getEncoder().getVelocity() > 10_000){
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