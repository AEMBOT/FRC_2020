package frc.robot.Hardware.Electrical;

import edu.wpi.first.wpilibj.I2C;

/**
 * Class used to interface with an I2C Multiplexer (for use with color sensors among other things)
 * 
 * @author Will Richards
 */
public class I2C_Multiplexer{

    // The default address that the Multiplexer starts at
    private final byte defaultAddress = 0x70;

    // Todo: Confirm Address is correct
    I2C i2c = new I2C(I2C.Port.kOnboard, 0x50);

    /**
     * Selects the port that is currently in use
     */
    public void selectPort(byte port){
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
            selectPort(i);
            System.out.println("Multiplex Port # " + i);

            //Loop through every address on the multiplexer
            for(byte addr=0; addr<=127; addr++){

                //Ignore the starting address
                if (addr == defaultAddress){
                    continue;
                }

                //Attempts to write to the specific I2C bus to see if it exists
                if(!i2c.write(addr, 0)){
                    System.out.println("Valid Bus Found!");
                }

            }
        }
    }
}