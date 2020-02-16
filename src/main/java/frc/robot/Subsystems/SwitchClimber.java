package frc.robot.Subsystems;

import frc.robot.RobotMap;
import frc.robot.Hardware.Pneumatics.DoublePiston;

/**
 * Fully inclusive climbing class (Winch, Deployment Method, etc.)
 */
public class SwitchClimber{

    //Climber pistons
    private DoublePiston climberPiston;

    /**
     * Construct the climber
     */
    public SwitchClimber(){
        climberPiston = new DoublePiston(RobotMap.ClimberPistonA, RobotMap.ClimberPistonB);
    
    }

    /**
     * Deploy the hook
     */
    public void deployClimber(){
        climberPiston.actuate();
    }

    public void retractClimber(){
        climberPiston.retract();
    }

}