package frc.robot.Hardware.Pneumatics;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;

/**
 * Special class used to control the compressor more effectively
 * 
 * @author Will Richards
 */
public class AdvancedCompressor{

    // Variable to hold the actual compressor object
    private static Compressor compressor;

    // Variable to hold the value of this class
    private static AdvancedCompressor advancedCompressor;

    /**
     * Construct the actual WPILIB compressor class
     */
    private AdvancedCompressor(){
        compressor = new Compressor();
    }

    /**
     * Get a static reference to the current compressor class
     * @return the reference to the compressor
     */
    public static AdvancedCompressor get(){
        if(advancedCompressor == null){
            advancedCompressor = new AdvancedCompressor();
        }

        return advancedCompressor;
    }

    /**
     * Starts running the actual compressor
     */
    public static void startCompressor(){
        get().compressor.start();
    }
    
    /**
     * Stop running the compressor
     */
    public static void stopCompressor(){
        get().compressor.stop();
    }

    /**
     * When called this method will turn the compressor on for a set amount of time and then turn it back off
     * @param seconds the time to keep the compressor on
     */
    public static void startTimedRecharge(double seconds){
        startCompressor();
        Timer.delay(seconds);
        stopCompressor();
    }
    
    /**
     * Return the status of the pressure switch
     * @return boolean for pressure switch
     */
    public static boolean getPressureSwitchStatus(){
       return get().compressor.getPressureSwitchValue();
    }

    /**
     * Clears all the faults on the PCM/Compressor
     */
    public static void clearFaults(){
        get().compressor.clearAllPCMStickyFaults();
    }

    /**
     * Returns the current current draw from the compressor
     * @return double value of the current draw
     */
    public static double getCurrent(){
        return get().compressor.getCompressorCurrent();
    }

    /**
     * Returns the current status of the compressor
     * @return boolean showing true / false for on or off
     */
    public static boolean getCompressorStatus(){
        return get().compressor.enabled();
    }

}