package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;
import frc.robot.Communication.Dashboard.Dashboard;

/**
 * Implementation of an Arc Shooter Teleop/Autonomous
 */
public class ArcShooter{

    // Motor that runs the shooter
    private CANSparkMax shooterMotor;

    // Weather or not the shooter should be running
    private boolean toggledStatus = false;

    // If the fly wheel is up to speed
    private boolean flywheelStatus = false;

    private double currentFlyWheelPower = 0.0;

    public ArcShooter(){
        shooterMotor = new CANSparkMax(RobotMap.ShooterTestMotor, MotorType.kBrushless);
        
        // Flip the normal direction of the motor
        shooterMotor.setInverted(true);
        
    }

    /**
     * Constantly called only executed when toggled, allows for the flywheel to ramp up or down to allow for smooth-ish curve
     */
    public void runShooter(){
        if(toggledStatus){
            if(currentFlyWheelPower < 1)
                currentFlyWheelPower += 0.05;
        }
        else{
            if(currentFlyWheelPower > 0.1){
                currentFlyWheelPower -= 0.05;
            }
            else{
                currentFlyWheelPower = 0;
            }
        }

        //Add the RPM values to the smart dashboard
        Dashboard.setValue("Fly-Wheel-RPM", shooterMotor.getEncoder().getVelocity());
        
        shooterMotor.set(currentFlyWheelPower);
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