package frc.robot;

/**
 * Class created to hold static references of physical robot constants
 */
public class RobotConstants {

    // Number of ticks per one revolution of the wheel, 8192 is based on a REV
    // through bore encoders
    public static final int TICKS_PER_REV = 8192;

    // Calculate the circumference of an 8in pneumatic wheel
    public static final double WHEEL_CIRCUMFERENCE = (Math.PI * 8);
}