package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.RobotMap;
import frc.robot.Hardware.Pneumatics.DoublePiston;

/**
 * Fully inclusive climbing class (Winch, Deployment Method, etc.)
 */
public class SwitchClimber{

    //Climber pistons
    private DoublePiston climberPiston;

    // Create the climber winches
    private TalonFX rightWinch;
    private TalonFX leftWinch;

    /**
     * Construct the climber
     */
    public SwitchClimber(){
        climberPiston = new DoublePiston(RobotMap.ClimberPistonA, RobotMap.ClimberPistonB);
    
        // Create the winch motors
        rightWinch = new TalonFX(RobotMap.RightWinchMotor);
        leftWinch = new TalonFX(RobotMap.LefWinchMotor);

        // Set the left winch to follow the right winch
        //leftWinch.follow(rightWinch, FollowerType.PercentOutput);
       rightWinch.setInverted(InvertType.InvertMotorOutput);

    }

    /**
     * Deploy the hook
     */
    public void deployClimber(){
        climberPiston.retract();
    }
    

    /**
     * Retract the climber
     */
    public void retractClimber(){
        climberPiston.actuate();
    }

    /**
     * Run the winch manually down
     * @param power the power to apply winch
     */
    public void manualWinch(double power){
        rightWinch.set(ControlMode.PercentOutput, power);
        leftWinch.set(ControlMode.PercentOutput, -power);
    }

    public void reverseWinch(double power){
        rightWinch.set(ControlMode.PercentOutput, power);
        leftWinch.set(ControlMode.PercentOutput, -power);
    }

    /**
     * Runs the left winch individually
     */
    public void runLeftWinch(double power){
        leftWinch.set(ControlMode.PercentOutput, power);
    }

    /**
     * Run the right winch individually
     */
    public void runRightWinch(double power){
        rightWinch.set(ControlMode.PercentOutput, power);
    }

}