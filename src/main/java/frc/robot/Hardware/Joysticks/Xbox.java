package frc.robot.Hardware.Joysticks;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Wrapper for an Xbox 360 Xbox. Provides clearer interface with button and axis inputs.
 * 
 * @author Patrick Higgins
 */
public class Xbox {

    public XboxController controller;

    /**
     * Constructor for Xbox.
     *
     * @param controller the xbox's joystick.
     */
    public Xbox(XboxController controller) {
        this.controller = controller;
    }

    /**
     * @return The value of the X axis of the left stick
     */
    public double leftStickX() {
        return controller.getRawAxis(0);
    }

    /**
     * @return The value of the Y axis of the left stick, reversed
     * so that up is positive and down is negative.
     */
    public double leftStickY() {
        return -controller.getRawAxis(1);
    }

    /**
     * @return The value of the X axis of the right stick.
     */
    public double rightStickX() {
        return controller.getRawAxis(4);
    }

    /**
     * @return The value of the Y axis of the right stick, reversed
     * so that up is positive and down is negative.
     */
    public double rightStickY() {
        return -controller.getRawAxis(5);
    }

    /**
     * @return The value of the axis for the left trigger.
     */
    public double leftTrigger() {
        return controller.getRawAxis(2);
    }

    /**
     * @return The value of the axis for the right trigger.
     */
    public double rightTrigger() {
        return controller.getRawAxis(3);
    }

    /**
     * @return the value of the left bumper.
     */
    public boolean leftBumper() {
        return controller.getRawButton(5);
    }

    /**
     * @return the value of the right bumper.
     */
    public boolean rightBumper() {
        return controller.getRawButton(6);
    }

    /**
     * @return the value of the left joystick button.
     */
    public boolean leftStickButton() {
        return controller.getRawButton(9);
    }

    /**
     * @return the value of the right joystick button.
     */
    public boolean rightStickButton() {
        return controller.getRawButton(10);
    }

    /**
     * @return the value of the A button.
     */
    public boolean A() {
        return controller.getRawButton(1);
    }

    /**
     * @return the value of the B button.
     */
    public boolean B() {
        return controller.getRawButton(2);
    }

    /**
     * @return the value of the X button.
     */
    public boolean X() {
        return controller.getRawButton(3);
    }

    /**
     * @return the value of the Y button.
     */
    public boolean Y() {
        return controller.getRawButton(4);
    }
    
    //PLEASE FIND THE ACTUAL NAME FOR THESE
    public boolean seven() {
        return controller.getRawButton(7);
    }

    public boolean eight() {
        return controller.getRawButton(8);
    }


    /**
     * @return if the dpad is pushed up
     */
    public boolean dPadUp(){
        return controller.getPOV(0) == 0;
    }

    /**
     * @return if the dpad is pushed down
     */
    public boolean dPadDown(){
        return controller.getPOV(0) == 180;
    }

    /**
     * @return if the dpad is pushed to the right
     */
    public boolean dPadRight(){
        return controller.getPOV(0) == 90;
    }
    /**
     * @return if the dpad is pushed to the right
     */
    public boolean dPadLeft(){
        return controller.getPOV(0) == 270;
    }

}
