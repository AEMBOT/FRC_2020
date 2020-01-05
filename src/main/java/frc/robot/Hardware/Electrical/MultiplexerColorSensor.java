package frc.robot.Hardware.Electrical;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Class used to interface with an I2C Multiplexer specifically for interaction with REV V3 Color Sensor
 * Ref: http://www.revrobotics.com/rev-31-1557/
 * 
 * @author Will Richards
 */
public class MultiplexerColorSensor{

    // The default address that the Multiplexer starts at
    private final byte defaultAddress = 0x70;

    // Todo: Confirm Address is correct
    I2C i2c = new I2C(I2C.Port.kOnboard, 0x50);

    //Init a color sensor on the I2C port
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    /**
     * Selects the port that is currently in use
     */
    public void selectPort(int port){
        if (port > 7){
            throw new RuntimeException("Multiplexer Port Out Of Range");
        }
        
        //Bit shift port by 1
        int shiftedPort = (1 << port);

        i2c.write(defaultAddress, shiftedPort);
    }

    /**
     * Used to find ports that have actively connected buses
     */
    public void discoverBuses(){

        // Loop through all 8 ports to find connected hexes
        for (byte i=0; i<8; i++){
            selectPort(Byte.toUnsignedInt(i));
            System.out.println("Multiplex Port # " + i);

            //Loop through every address on the multiplexer
            for(byte addr=0; addr<=127; addr++){

                //Ignore the starting address
                if (Byte.toUnsignedInt(addr) == defaultAddress){
                    continue;
                }

                //Attempts to write to the specific I2C bus to see if it exists, if so return the buses address in hex
                if(!i2c.write(addr, 0)){
                    System.out.println("Valid Bus Found At " + Integer.toHexString(Byte.toUnsignedInt(addr)));
                }

            }
        }
    }


    /**
     * Reads the color from the sensor\
     * @param port
     */
    public Color readColor(int port){

        //Change to the correct I2C port
        selectPort(port);

        return colorSensor.getColor();
    }
}