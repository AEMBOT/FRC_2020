package frc.robot.Hardware.Sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

/**
 * Wrapper class used to interface with the NavX
 * 
 * @author Will Richards
 */
public class NavX {
    public static NavX instance;

    private AHRS ahrs;

    /**
     * Creates a private constructor so that this class cannot be created more than
     * once
     */
    private NavX() {
        try {
            ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
        }
    }

    /**
     * Creates a global reference to the singular NavX
     */
    public static NavX get() {
        if (instance == null) {
            instance = new NavX();
        }
        return instance;
    }

    /**
     * Gets the yaw axis or for all intensive purposes the robot heading purely
     * based on the gyro scope inside (0-180, -180-0)
     * 
     * @return the robot heading
     */
    public float getYaw() {
        return ahrs.getYaw();
    }

    /**
     * Get the correct yaw to 0-360
     * 
     * @return the correct yaw
     */
    public double getCorrectedHeading() {
        double yaw = ahrs.getYaw();
        if (yaw >= 0) {
            return yaw;
        } else {
            return (360 - Math.abs(yaw));
        }
    }

    public double getEdgeCaseAngle(double setpoint){
        return (((((setpoint - getCorrectedHeading()) + 180)+360) % 360) - 180);
    }

    /**
     * This method can't truly be reset and will return a heading based of the gyro
     * and the compass (0-360)
     * 
     * @return
     */
    public float getFusedHeading() {
        return ahrs.getFusedHeading();
    }

    /**
     * Reset the yaw axis on the robot
     */
    public void resetYaw(){
        ahrs.zeroYaw();
    }

    /**
     * !Warning! The value may not be accurate due to EMI from the motors Gets the
     * value from the compass in the NavX
     * 
     * @return compass heading
     */
    public float getCompassHeading() {
        return ahrs.getCompassHeading();
    }

    /**
     * Used to reset the robot heading on the NavX
     */
    public void reset() {
        ahrs.reset();
    }

    /**
     * Determines whether or not the robot is moving
     * 
     * @return whether or not the robot is moving
     */
    public boolean isMoving() {
        return ahrs.isMoving();
    }

    /**
     * Determines whether or not the robot is rotating
     * 
     * @return whether or not the robot is rotating
     */
    public boolean isRotating() {
        return ahrs.isRotating();
    }

    /**
     * Return the current state of the NavX calibration
     * 
     * @return whether or not the robot is actively calibrating
     */
    public boolean isCalibrating() {
        return ahrs.isCalibrating();
    }

    public double getCorrectAngle(){
        return Math.IEEEremainder(getAngle(), 360) * -1;
    }

    /**
     * Return the rate at which the NavX is moving
     * @return rate of change in seconds
     */
    public double getRate(){
        return ahrs.getRate();
    }

    /**
     * Return a continuos angle greater than 360 degrees
     * @return the total angle
     */
    public double getAngle(){
        return ahrs.getAngle();
    }
    public AHRS getAhrs(){
        return ahrs;
    }
}