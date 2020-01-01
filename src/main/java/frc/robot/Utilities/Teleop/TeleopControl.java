package frc.robot.Utilities.Teleop;

import java.util.ArrayList;

import frc.robot.Hardware.Joysticks.Xbox;

/**
 * A modified variant of the TeleopStructure.java file used in previous years (original author: Lizzy Milford)
 * @author Will Richards
 */
public class TeleopControl{

    //Signifies wheather or not its the first loop of the teleop periodic, used to init all the values
    private boolean firstLoop = true;

    //The ID used to designate which method is currently being used, keeps track of button values
    private int oncePerPressMethodID = 0;
    private int toggleMethodID = 0;

    //List to hold values for each runOncePerPress instance, effectively keeping track of all buttons
    private ArrayList<Boolean> pressedStatusList = new ArrayList<>();
    private ArrayList<Boolean> toggleStatusList = new ArrayList<>();

    /**
     * Run the method once when the button is first pressed
     * @param button the status of the pressed button
     * @param action the method that is run when the button is pressed
     */
    public void runOncePerPress(boolean button, Runnable action){
        
        //Init all the button statues, simply add it to the list in the order they appear in Robot.java
        if(firstLoop){
            pressedStatusList.add(false);
        }

        //If the button is currently pressed
        if(button){

            //Check that the current button has NOT been pressed already, if so run the function
            if(pressedStatusList.get(oncePerPressMethodID) == false){
                action.run();
            }

            //Now that it has been run once change the value in the list to true signifying that it has been run
            pressedStatusList.set(oncePerPressMethodID, true);
        }
        else{

            //Now that the button has been released we can set the button back to unpressed
            pressedStatusList.set(oncePerPressMethodID, false);
        }

        //Increment the ID by one to 
        oncePerPressMethodID++;
    }

    /**
     * While the current button is pressed continually
     * @param button the status of the pressed button
     * @param action the method that is run when the button is pressed
     */
    public void pressed(boolean button, Runnable action){
        if(button){
            action.run();
        }
    }

    /**
     * Run a certain action whenever a certain button isn't pressed
     * @param button the button to use
     * @param action the action to preform
     */
    public void notPressed(boolean button, Runnable action){
        if(!button){
            action.run();
        }
    }


    /**
     * Used to toggle running a function on or off, called from the overhead method toggle which utilizes the run once per press method
     * @param button the button used to preform the operation
     * @param action the action to preform
     */
    private void toggling(Runnable action){

        //Init the amount of toggle calls into the list
        if(firstLoop){
            toggleStatusList.add(false);
        }

        //Swap the current value of the toggle
        toggleStatusList.set(toggleMethodID, !toggleStatusList.get(toggleMethodID));

        //If it is supposed to be looping call the method
        if(toggleStatusList.get(toggleMethodID) == true){
            action.run();
        }

        toggleMethodID++;
    }

    /**
     * Called to tell the toggling method to actually toggle the method, because its easier to use a run once per press than not
     * @param button the value of the button to feed into the runOncePerPress method
     * @param action the action to preform
     */
    public void toggle(boolean button, Runnable action){
        runOncePerPress(button, () -> toggling(action));
    }

   /**
    * Used to reset methodIDS so we can check the values again
    */
   public void endPeriodic(){
        firstLoop = false;
        oncePerPressMethodID = 0;
        toggleMethodID = 0;
   }
}