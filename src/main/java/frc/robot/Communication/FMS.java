package frc.robot.Communication;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Communication.Enums.Color;

/**
 * Class used to interface with the FMS / Driver Station
 * 
 * @author Will Richards
 */
public class FMS {

    /**
     * Get the alliance the robot is currently on
     * 
     * @return the robot's alliance
     */
    public static DriverStation.Alliance getAlliance() {
        return DriverStation.getInstance().getAlliance();
    }

    /**
     * Get the current robot battery voltage
     * 
     * @return the battery voltage
     */
    public static double getBatteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    /**
     * Get the current event's name
     * 
     * @return the event name
     */
    public static String getEventName() {
        return DriverStation.getInstance().getEventName();
    }

    /**
     * Get the game message from the FMS
     * 
     * @return the game message
     */
    public static String getGameMessage() {
        return DriverStation.getInstance().getGameSpecificMessage();
    }

    /**
     * Gets the specific control panel color from the FMS
     * @return the color of the control panel
     */
    public static Color getControlPanelColor(){
        String data = getGameMessage();

        // Verify there is actual data there
        if(data.length() > 0){
            switch(data.charAt(0)){
                case 'B':
                    return Color.BLUE;
                case 'G':
                    return Color.GREEN;
                case 'Y':
                    return Color.YELLOW;
                case 'R':
                    return Color.RED;
                default:
                    return Color.BAD_DATA;
            }
        }
        
        return Color.NO_DATA;
    }

    /**
     * Get the current match number
     * 
     * @return the match number
     */
    public static int getMatchNumber() {
        return DriverStation.getInstance().getMatchNumber();
    }

    /**
     * Get the current match time
     * 
     * @return the match time
     */
    public static double getMatchTime() {
        return DriverStation.getInstance().getMatchTime();
    }

    /**
     * Get whether or not the robot is actively browning out
     * 
     * @return brown out status
     */
    public static boolean isBrowningOut() {
        return RobotController.isBrownedOut();
    }

}