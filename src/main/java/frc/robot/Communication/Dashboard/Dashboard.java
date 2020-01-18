package frc.robot.Communication.Dashboard;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.function.Consumer;

/**
 * Wrapper style class for the network tables for use with Shuffleboard
 * 
 * @author Will Richards
 */
public class Dashboard {

    // Create an instance of a network table to interface with th
    private NetworkTableInstance instance;

    // A refrence to the appropriate network table, default to SmartDashboard
    private static NetworkTable networkTable;

    private static String tableName = "SmartDashboard";

    private static ArrayList<String> entryList;

    // Values for active listeners
    private static ArrayList<Integer> listenerHandlerList;

    /**
     * Get a static reference to the network table
     */
    private static NetworkTableInstance getInstance() {
        return NetworkTableInstance.getDefault();
    }

    /**
     * Sets an alternative table to be used other than SmartDashboard
     */
    public static void setTable(String tableNameParam) {

        // Make sure the table name is valid
        if (tableNameParam.equals("SmartDashboard") || tableNameParam.equals("LiveWindow")) {
            networkTable = getInstance().getTable(tableNameParam);
            tableName = tableNameParam;
        } else {
            throw new RuntimeException("Invalid Table Name, Use \"SmartDashboard\" or \"LiveWindow\"");
        }

        // List of all the entries existing in the current table
        entryList = new ArrayList<>();
        listenerHandlerList = new ArrayList<>();
    }

    /**
     * Create an entry in the selected table
     * 
     * @param entryName the name of the entry
     */
    public static void createEntry(String entryName, Object defaultValue) {
        networkTable.getEntry(entryName);
        entryList.add(networkTable.getEntry(entryName).getName());
        listenerHandlerList.add(0);
        networkTable.getEntry(entryName).setValue(defaultValue);
    }

    /**
     * Create an entry in the selected table
     * 
     * @param entryName the name of the entry
     */
    public static void createEntry(String entryName) {
        networkTable.getEntry(entryName);
        entryList.add(networkTable.getEntry(entryName).getName());
        listenerHandlerList.add(0);
    }

    /**
     * Sets a value to a given entry
     * 
     * @param entryName the entry to affect
     * @param value     vague variable that allows multiple types
     */
    public static void setValue(String entryName, Object value) {
        try {
            networkTable.getEntry(entryName).setValue(value);
        } catch (IllegalArgumentException e) {
            SmartDashboard.putData(entryName, (Sendable) value);
        }
    }

    /**
     * Converts the method to a command and adds that to the dashboard
     * 
     * @param entryName where we want to add it
     * @param method    the method we want to add
     */
    public static void addRunableMethod(String entryName, Runnable method) {
        SmartDashboard.putData(entryName, new Run(method));
    }

    /**
     * Get the value of a give entry
     * 
     * @param entryName the name of the entry to get the value from
     * @return the value of the entry
     */
    public static Object getValue(String entryName) {
        return networkTable.getEntry(entryName).getValue();
    }

    /**
     * Adds A PID Controller to the 
     * @param entryName the location to store it
     * @param updateFunction the function to call when the values are changed
     */
    public static void createPIDController(String entryName){
        if(!SmartDashboard.containsKey(entryName)){
            PIDController pid = new PIDController(0, 0, 0);
            SmartDashboard.putData(entryName, pid);
        }
    }

    /**
     * Clears persistent data for a set key
     */
    public static void clearPersistance(String entryName){
        SmartDashboard.clearPersistent(entryName);
    }

    /**
     * Enable saving of data between restarts
     * @param entryName
     */
    public static void enablePersistance(String entryName){
        SmartDashboard.setPersistent(entryName);
    }

    /**
     * Returns the PID values from a controller
     * @param entryName
     * @return the PID values in an array
     */
    public static double[] getPID(String entryName){
        double[] pidConstants = new double[3];

        PIDController pid = (PIDController) SmartDashboard.getData(entryName);

        //P
        pidConstants[0] = pid.getP();

        //I
        pidConstants[1] = pid.getI();

        //D
        pidConstants[2] = pid.getD();

        return pidConstants;
    }

    /**
     * A method used to easily setup entry listeners
     * 
     * @param entryName the name of the entry to listen on
     */
    public static void setUpEntryListener(String entryName, Consumer<NetworkTableValue> updateFunction) {

        // Create the listner and add it to the correct spot in the listener list
        listenerHandlerList.set(entryList.indexOf("/" + tableName + "/" + entryName),
                networkTable.addEntryListener(entryName, new TableEntryListener() {
                    @Override
                    public void valueChanged(NetworkTable networkTable, String s, NetworkTableEntry networkTableEntry,
                            NetworkTableValue networkTableValue, int i) {
                        updateFunction.accept(networkTableValue);
                    }
                }, EntryListenerFlags.kUpdate));

    }

    /**
     * A method used to add a table wide listener
     */
    public static void removeEntryListener(String entryName) {

        // Remove the listener from the corresponding list
        networkTable.getEntry(entryName)
                .removeListener(listenerHandlerList.get(entryList.indexOf("/" + tableName + "/" + entryName)));

        // Reset the handler number back to 0
        listenerHandlerList.set(entryList.indexOf("/" + tableName + "/" + entryName), 0);
    }
}
