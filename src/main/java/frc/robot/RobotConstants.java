package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * Class created to hold static references of physical robot constants
 */
public class RobotConstants {

    // Number of ticks per one revolution of the wheel, 8192 is based on a REV
    // through bore encoders
    public static final int TICKS_PER_REV = 8192;

    // Calculate the circumference of an 8in pneumatic wheel
    public static final double WHEEL_CIRCUMFERENCE = (Math.PI * 8);

    /**
     * Robot Characterization Link:
     * https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-characterization/introduction.html#installing-and-launching-the-toolsuite
     * The following section of variables is specifically for use with the S-Curve
     * Trajectories TODO: Find values for 2020 bot drive train
     */

    // The scale factor required to convert encoder pulses to meters
    public static final double kEncoderDistancePerPulse = -1;

    public static final double kMaxUsableVoltage = 10;

    public static final boolean kGyroReversed = false;

    // Voltage for static friction velocity and acceleration
    public static final double kSVolts = -1;
    public static final double kvVoltMetersPerSecond = -1;
    public static final double kaVoltMetersPerSecondSquared = -1;

    // PID values (Only P is required for velocity)
    public static final double kPDriveVal = -1;

    // Kinematic information about our robot
    public static final double kTrackWidthMeters = -1;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackWidthMeters);

    // Sets values for the speed at which we will reach the max velocity and what
    // the max velocity
    public static final double kMaxVelocityMetersPerSecond = -1;
    public static final double kMaxAccelerationMetersPerSecondSquared = -1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

}