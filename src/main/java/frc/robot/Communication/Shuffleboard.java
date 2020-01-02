package frc.robot.Communication;

import edu.wpi.first.networktables.*;

import java.util.ArrayList;
import java.util.function.Consumer;

/**
 * Wrapper style class for the network tables for use with Shuffleboard
 * @author Will Richards
 */
public class Shuffleboard {

    //Create an instance of a network table to interface with th
    private NetworkTableInstance instance;
    
    //A refrence to the appropriate network table, default to SmartDashboard
    private static NetworkTable networkTable = getInstance().getTable("SmartDashboard");;

    private static String tableName = "SmartDashboard" ;

    private static ArrayList<String> entryList;

    //Values for active listeners
    private static ArrayList<Integer> listenerHandlerList;

    /**
     * Get a static reference to the network table
     */
    private static NetworkTableInstance getInstance(){
        return NetworkTableInstance.getDefault();
    }

    /**
     * Sets an alternative table to be used other than SmartDashboard
     */
    public static void setTable(String tableNameParam){

        //Make sure the table name is valid
        if(tableNameParam.equals("SmartDashboard") || tableNameParam.equals("LiveWindow")){
            networkTable = getInstance().getTable(tableNameParam);
            tableName = tableNameParam;
        }
        else {
            throw new RuntimeException("Invalid Table Name, Use \"SmartDashboard\" or \"LiveWindow\"");
        }

        //List of all the entries existing in the current table
        entryList = new ArrayList<>();
        listenerHandlerList = new ArrayList<>();
    }

    /**
     * Create an entry in the selected table
     * @param entryName the name of the entry
     */
    public static void createEntry(String entryName){
        networkTable.getEntry(entryName);
        entryList.add(networkTable.getEntry(entryName).getName());
        listenerHandlerList.add(0);
    }

    /**
     * Sets a value to a given entry
     * @param entryName the entry to affect
     * @param value vague variable that allows multiple types
     */
    public static void setValue(String entryName, Object value){
        networkTable.getEntry(entryName).setValue(value);
    }

    /**
     * Get the value of a give entry
     * @param entryName the name of the entry to get the value from
     * @return the value of the entry
     */
    public static Object getValue(String entryName){
        return networkTable.getEntry(entryName).getValue();
    }

    /**
     * A method used to easily setup entry listeners
     * @param entryName the name of the entry to listen on
     */
    public static void setUpEntryListener(String entryName, Consumer<NetworkTableValue> updateFunction){

        //Create the listner and add it to the correct spot in the listener list
        listenerHandlerList.set(entryList.indexOf("/"+tableName+"/"+entryName),
        networkTable.addEntryListener(entryName, new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable networkTable, String s, NetworkTableEntry networkTableEntry, NetworkTableValue networkTableValue, int i) {
                updateFunction.accept(networkTableValue);
            }
        }, EntryListenerFlags.kUpdate));

    }

    /**
     * A method used to add a table wide listener
     */
    public static void removeEntryListener(String entryName){

        //Remove the listener from the corresponding list
        networkTable.getEntry(entryName).removeListener(listenerHandlerList.get(entryList.indexOf("/"+tableName+"/"+entryName)));

        //Reset the handler number back to 0
        listenerHandlerList.set(entryList.indexOf("/"+tableName+"/"+entryName), 0);
    }
}
