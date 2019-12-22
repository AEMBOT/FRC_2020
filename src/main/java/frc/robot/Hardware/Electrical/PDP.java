package frc.robot.Hardware.Electrical;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * Class used to interface with the PDP to get electrical read outs
 * @author Will Richards
 */
public class PDP{

    private PowerDistributionPanel pdp;

    /**
     * Initialize the PDP to return values from
     */
    public PDP(){
        pdp = new PowerDistributionPanel(0);
    }

    /**
     * Get the current on a specific channel of the PDP
     * @param channelNum the channel number
     * @return the being drawn from the specific channel
     */
    public double getCurrent(int channelNum){
        return pdp.getCurrent(channelNum);
    }

    /**
     * Gets the voltage currently required by the PDP
     * @return the voltage required
     */
    public double getVoltage(){
        return pdp.getVoltage();
    }

    /**
     * Get the internal temperature of the PDP
     * @return the temperature of the PDP
     */
    public double getTemperature(){
        return pdp.getTemperature();
    }

    /**
     * Return the total wattage of the PDP
     * @return the total wattage
     */
    public double getTotalWattage(){
        return pdp.getTotalEnergy();
    }
}