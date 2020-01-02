package frc.robot.Hardware.Joysticks;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Class used to handle joystick input from the Logitech Extreme 3D Pro
 */
public class Extreme3DPro {

    public Joystick controller;

    public Extreme3DPro(Joystick controller) {
        this.controller = controller;
    }

    /**
     * @return The value of the X axis.
     */
    public double xAxis() {
        return controller.getRawAxis(0);
    }

    /**
     * @return The value of the Y axis.
     */
    public double yAxis() {
        return controller.getRawAxis(1);
    }

    /**
     * @return The value of the Z axis (rotation).
     */
    public double zAxis() {
        return controller.getRawAxis(2);
    }

    /**
     * @return The value of the slider.
     */
    public double slider() {
        return controller.getRawAxis(3);
    }

    /**
     * @return the value of the trigger.
     */
    public boolean trigger() {
        return controller.getRawButton(1);
    }

    /**
     * @return the value of the thumb button.
     */
    public boolean thumb() {
        return controller.getRawButton(2);
    }

    /**
     * @return the value of the three button.
     */
    public boolean three() {
        return controller.getRawButton(3);
    }

    /**
     * @return the value of the four button.
     */
    public boolean four() {
        return controller.getRawButton(4);
    }

    /**
     * @return the value of the five button.
     */
    public boolean five() {
        return controller.getRawButton(5);
    }

    /**
     * @return the value of the six button.
     */
    public boolean six() {
        return controller.getRawButton(6);
    }

    /**
     * @return the value of the seven button.
     */
    public boolean seven() {
        return controller.getRawButton(7);
    }

    /**
     * @return the value of the eight button.
     */
    public boolean eight() {
        return controller.getRawButton(8);
    }

    /**
     * @return the value of the nine button.
     */
    public boolean nine() {
        return controller.getRawButton(9);
    }

    /**
     * @return the value of the ten button.
     */
    public boolean ten() {
        return controller.getRawButton(10);
    }

    /**
     * @return the value of the eleven button.
     */
    public boolean eleven() {
        return controller.getRawButton(11);
    }

    /**
     * @return the value of the twelve button.
     */
    public boolean twelve() {
        return controller.getRawButton(12);
    }
}
