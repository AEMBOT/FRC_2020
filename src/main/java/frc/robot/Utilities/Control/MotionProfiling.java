package frc.robot.Utilities.Control;

import edu.wpi.first.wpilibj.Timer;

/*
 * Equations from ChiefDelphi user 'Ether' (Original Is no longer in existence):
 * https://web.archive.org/web/20181227133106/https://www.chiefdelphi.com/media/papers/download/4496
 *
 * Desmos graph of distance, velocity, and acceleration (modify at will. Example values: D = 5, MaxAcceleration = 3.5):
 * https://www.desmos.com/calculator/la5fv4ohqy
 *
 * Use the MotionProfile class with a PID. Feed the getDistance() (or speed or acceleration) into
 * the setDesiredValue() of the PID.
 */
public class MotionProfiling {

    private Timer timer;
    private double T; // time to destination
    private double K1;
    private double K2;
    private double K3;
    private double MaxAcceleration;
    private double maxSpeed;

    /**
     * Create a new motion profiling object given the distance we want to move and
     * the max acceleration to get there
     * 
     * @param distance        wanted distance
     * @param maxAcceleration the maximum acceleration
     */
    public MotionProfiling(double distance, double maxAcceleration) {
        timer = new Timer();
        calculate(distance, maxAcceleration);
    }

    /**
     * Used to calculate good values for max acceleration
     * 
     * @param distance        the wanted distance
     * @param maxAcceleration the max acceleration
     * @param desiredMaxSpeed and the desired max speed
     */
    public MotionProfiling(double distance, double maxAcceleration, double desiredMaxSpeed) {
        this(distance, maxAcceleration);

        // Check if the max speed is less than desired
        if (maxSpeed <= desiredMaxSpeed)
            System.out.println("Max speed calculations correct. Max speed: " + maxSpeed);

        // While maxSpeed is still too big for the desired max speed recalculate the
        // acceleration
        while (maxSpeed > desiredMaxSpeed) {
            double newMaxAcceleration;
            System.out.println("Max speed higher than desired max speed!");
            newMaxAcceleration = (Math.pow(desiredMaxSpeed, 2)
                    - Math.pow(desiredMaxSpeed - maxSpeed, 2) / (2 * Math.PI * distance));
            System.out.println("Changing max acceleration to: " + newMaxAcceleration);
            calculate(distance, maxAcceleration);
        }
    }

    // calculates constants. DO NOT MODIFY THIS!!!
    private void calculate(double distance, double maxAcceleration) {
        MaxAcceleration = maxAcceleration;

        // Calculates the time to destination
        T = Math.sqrt((2 * Math.PI * distance) / MaxAcceleration);

        K1 = (2 * Math.PI) / T;
        K2 = MaxAcceleration / K1;
        K3 = 1 / K1;
        maxSpeed = 2 * K2;
    }

    /**
     * Starts the timer which in turn starts the calculations
     */
    public void startTimer() {
        timer.start();
    }

    /**
     * Stop updating distances
     */
    public void stopTimer() {
        timer.stop();
    }

    /**
     * Reset the time before use again
     */
    public void resetTimer() {
        timer.reset();
    }

    /**
     * Get the current time since start
     */
    public double getTime() {
        return timer.get();
    }

    /**
     * get the acceleration you should be at at timer.get()
     */
    public double getAcceleration() {
        return MaxAcceleration * Math.sin(K1 * timer.get());
    }

    /**
     * Get speed is the speed you should be at timer.get()
     */
    public double getSpeed() {
        return K2 * (1 - Math.cos(K1 * timer.get()));
    }

    /**
     * Get distance where you should be at timer.get()
     */
    public double getDistance() {
        return K2 * (timer.get() - K3 * Math.sin(K1 * timer.get()));
    }

    /**
     * Print the expected time at a point
     */
    public void printExpectedTime() {
        System.out.println("Motion profile, expected time: " + T);
    }

    /**
     * Print the calculated max speed
     */
    public void printMaxSpeed() {
        System.out.println("Motion profile, max speed: " + maxSpeed);
    }

    /**
     * Helps to check values that feed into max speed
     * 
     * @param desiredMaxSpeed the max speed
     */
    public void printMaxSpeed(double desiredMaxSpeed) {
        if (maxSpeed > desiredMaxSpeed) {
            System.out.println("Max speed higher than desired max speed!");
            System.out.println("Motion profile, desired  max speed: " + desiredMaxSpeed);
        }
        System.out.println("Motion profile, max speed: " + maxSpeed);
    }
}