package frc.robot.Hardware.Electrical;

import java.io.ObjectInputFilter.Status;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * Class used to interface with the PDP to get electrical read outs
 * 
 * @author Will Richards
 */
public class PDP {

    private static PowerDistributionPanel pdp = new PowerDistributionPanel(0);

    /**
     * Get the current on a specific channel of the PDP
     * 
     * @param channelNum the channel number
     * @return the being drawn from the specific channel
     */
    public static double getCurrent(int channelNum) {
        return pdp.getCurrent(channelNum);
    }

    /**
     * Gets a reference to the PDP object
     */
    public static PowerDistributionPanel getInstance(){
        return pdp;
    }

    /**
     * Gets the voltage currently required by the PDP
     * 
     * @return the voltage required
     */
    public static double getVoltage() {
        return pdp.getVoltage();
    }

    /**
     * Get the internal temperature of the PDP
     * 
     * @return the temperature of the PDP
     */
    public static double getTemperature() {
        return pdp.getTemperature();
    }

    /**
     * Return the total wattage of the PDP
     * 
     * @return the total wattage
     */
    public static double getTotalWattage() {
        return pdp.getTotalEnergy();
    }

    /**
     * Clears Sticky Faults on PDP
     */
    public static void clearStickyFaults(){
        pdp.clearStickyFaults();
    }
}