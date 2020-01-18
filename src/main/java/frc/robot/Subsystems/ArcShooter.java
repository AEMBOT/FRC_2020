package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;
import frc.robot.Communication.Dashboard.Dashboard;

/**
 * Implementation of an Arc Shooter
 */
public class ArcShooter{

    // Motor that runs the shooter
    private CANSparkMax shooterMotor;

    // Weather or not the shooter should be running
    private boolean toggledStatus = false;

    // If the fly wheel is up to speed
    private boolean flywheelStatus = false;

    // The power to give to the flywheel motor
    private double currentFlyWheelPower = 0.0;

    public ArcShooter(){

        // Instance of the shooter motor
        shooterMotor = new CANSparkMax(RobotMap.ShooterTestMotor, MotorType.kBrushless);
        
        // Flip the normal direction of the motor
        shooterMotor.setInverted(true);
        
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

        //Add the RPM values to the smart dashboard
        Dashboard.setValue("Fly-Wheel-RPM", shooterMotor.getEncoder().getVelocity());
        
        //Inform the user of wheater or not the motor is up to speed
        if(shooterMotor.getEncoder().getVelocity() > 6000){
            Dashboard.setValue("Fly-Wheel-Speed-Status", true);
        }
        else{
            Dashboard.setValue("Fly-Wheel-Speed-Status", false);
        }

        Dashboard.setValue("Shooter-Current-Draw", getMotorCurrent());

        System.out.println(getMotorCurrent()[0]);

        //Set the flywheel speed to the ramped speed
        shooterMotor.set(currentFlyWheelPower);
    }

    /**
     * Returns the output current of the shooter motor as an array so it can be put onto the dashboard
     */
    public double[] getMotorCurrent(){
        double [] shooterCurrent = new double[1];
        shooterCurrent[0] = shooterMotor.getOutputCurrent();

        return shooterCurrent;
    }

    /**
     * Run the shooter motor given a manual power
     */
    public void manualShooter(double power){
        shooterMotor.set(power);
    }

    /**
     * Flip the value of the shooting variable
     */
    public void toggleShooter(){
        toggledStatus = !toggledStatus;
    }
}