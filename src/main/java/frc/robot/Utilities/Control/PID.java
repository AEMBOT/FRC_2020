package frc.robot.Utilities.Control;

/**
 * General class for implementing PID control into robot control
 * @author Will Richards
 */
public class PID {

    //PID Tuning constants
    private double P;
    private double I;
    private double D;

    //Values that are to be scaled by the PID constants
    private double p;
    private double i;
    private double d;

    //The last p value (error)
    private double lastError;

    //The value returned on calcOutput
    private double output;

    //Weather or not the current value is in the acceptable range and then what that acceptable range is
    private boolean inRange;
    private double acceptableRange;

    //Variable that determines whether or not to account for edge cases in this loop
    private boolean accountOverflow;

    //The point the module is trying to reach
    private double setpoint;

    //The max output PID can create, default 1
    private double maxOutput = 1;

    /**
     * Initialize the variables as well as assign PID constants
     * When accountOverflow is being used the getCorrectYaw method must also be used
     * @param P the P scalar
     * @param I the I scalar
     * @param D the D scalar
     * @param accountOverflow will alter the value to account for edge cases
     */
    public PID(double P, double I, double D, boolean accountOverflow){

        this.accountOverflow = accountOverflow;

        this.P = P;
        this.I = I;
        this.D = D;

        p = 0;
        i = 0;
        d = 0;

        lastError = 0;

        inRange = false;
        acceptableRange = 0;

        output = 0;
    }

    /**
     * This method is used to set the target for the PID loop
     * @param target the point the loop is trying to get to
     */
    public void setSetpoint(double target){
        setpoint = target;
    }

    /**
     * Sets the range that is considered in the right spot
     * @param range the acceptable range
     */
    public void setAcceptableRange(double range){
        acceptableRange = range;
    }

    /**
     * Returns weather or not the wanted position is within the set range
     * @return true / false accordingly
     */
    public boolean isInRange(){
        return inRange;
    }

    /**
     * Method used for setting the max output of the swerve module
     * @param maxOutput
     */
    public void setMaxOutput(double maxOutput){
        this.maxOutput = maxOutput;
    }

    /**
     * This method runs all the PID code to output a value to send to the motors
     * @return motor power
     */
    public double calcOutput(double currentValue){

        if (accountOverflow)
        {
            //Calculate the error to account for overflow
            p = (((setpoint-currentValue)+180) % 360)-180;
        }
        else{

            //Calculate the difference / error
            p = setpoint - currentValue;
        }

        //Error over time
        i += p;

        //Error rate of change
        d = p - lastError;
        lastError = p;

        //If we are trying to account for the overflow of the turing
        if(accountOverflow){

            //Use the output of error in the PID loop to determine if its in range (eg. P is only outputting a -1 or a 1 then stop)
            //Using 0 because that is the point the loop is technically trying to get to
            if(p > 0-acceptableRange && p < 0+acceptableRange){
                inRange = true;
                output = (P * p + I * i + D * d);
            }
            else{
                inRange = false;
                output = (P * p + I * i + D *d);
            }
            
        }
        else{
            //If the value is in the acceptable range set the flag to true
            if(currentValue > setpoint-acceptableRange && currentValue < setpoint+acceptableRange) {
                inRange = true;
                output = (P * p + I * i + D * d);
            }
            else{
                inRange = false;
                output = (P * p + I * i + D *d);
            }
        }

        //Scales the value to the max output
        if(Math.abs(output) > maxOutput){
            if(output<0)
                return -maxOutput;
            else
                return maxOutput;
        }

        return output;
    }

}