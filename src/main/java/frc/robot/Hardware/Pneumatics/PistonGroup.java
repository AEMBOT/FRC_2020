package frc.robot.Hardware.Pneumatics;

/**
 * Basic class used to hold a group of pistons to actuate as one
 * 
 * @author Will Richards
 */
public class PistonGroup{

    DoublePiston[] doublePistonArray;

    /**
     * Add as many double pistons to the array as passed into the parameters
     * @param doublePistons array of all pistons passed in
     */
    public PistonGroup(DoublePiston ...doublePistons){
        doublePistonArray = doublePistons;
    }

    /**
     * Loop through all pistons and actuate all of them
     */
    public void actuate(){
        for(DoublePiston piston : doublePistonArray){
            piston.actuate();
        }
    }

    /**
     * Loop through all pistons and retract them all
     */
    public void retract(){
        for(DoublePiston piston : doublePistonArray){
            piston.retract();
        }
    }
}