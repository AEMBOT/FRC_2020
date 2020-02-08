package frc.robot.Hardware.Pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Class used to implement a double action piston, in place of using a raw solenoid object
 * If you aren't using an actual piston just use the double solenoid class
 * 
 * @author Will Richards
 */
public class DoublePiston{

    // Double solenoid to control the piston
    private DoubleSolenoid solenoidPiston;

    /***
     * Constructs the solenoid / piston using 2 ports
     * @param forwardPort forward point on the solenoid
     * @param reversePort reverse point on the solenoid
     */
    public DoublePiston(int forwardPort, int reversePort){
        solenoidPiston = new DoubleSolenoid(forwardPort, reversePort);

        // Keep the current piston retracted at start so it doesn't fall outside the frame parameter
        retract();
    }

    /**
     * Extend the piston
     */
    public void actuate(){
        solenoidPiston.set(Value.kForward);
    }

    /**
     * Retract the piston
     */
    public void retract(){
        solenoidPiston.set(Value.kReverse);
    }

    /**
     * Sets the solenoid that controls the piston to closed and doesn't allow air to flow through
     */
    public void closeSolenoid(){
        solenoidPiston.set(Value.kOff);
    }

    /**
     * Gets the current value of the piston solenoid (kOff, kForward, kClose)
     */
    public Value getCurrentPistonState(){
        return solenoidPiston.get();
    }



}