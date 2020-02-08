package frc.robot.Subsystems;

import edu.wpi.first.hal.CompressorJNI;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;

public class Pnuematics{

    private DoubleSolenoid doubleSolenoid1;
    private DoubleSolenoid doubleSolenoid2;

    // Double Solenoid 1 Ports
    private final int doubleSolenoid1_forward = 0;
    private final int doubleSolenoid1_reverse = 1;

    // Double Solenoid 2 Ports
    private final int doubleSolenoid2_forward = 2;
    private final int doubleSolenoid2_reverse = 3;

    Compressor compressor;


    public Pnuematics(){
        doubleSolenoid1 = new DoubleSolenoid(doubleSolenoid1_forward, doubleSolenoid1_reverse);
        doubleSolenoid2 = new DoubleSolenoid(doubleSolenoid2_forward, doubleSolenoid2_reverse);

        doubleSolenoid1.set(Value.kForward);
        doubleSolenoid2.set(Value.kForward);


       compressor = new Compressor();


    }

    public void holdPressure(){
        compressor.start();
        Timer.delay(0.75);
        compressor.stop();
        
    }

    /**
     * Open Solenoid an actuate the piston 1
     */
    public void actuatePiston1(){
        doubleSolenoid1.set(Value.kForward);
        holdPressure();
    }

    public void startCompressor(){
       
           compressor.start();
       
        
    }

    public void stopCompressor(){
        compressor.stop();
    }

    /**
     * Retract piston 1
     */
    public void retractPiston1(){
        doubleSolenoid1.set(Value.kReverse);
        holdPressure();
    }

    /**
     * Open Solenoid an actuate the piston 1
     */
    public void actuatePiston2(){
        doubleSolenoid2.set(Value.kForward);
    }

    /**
     * Retract piston 1
     */
    public void retractPiston2(){
        doubleSolenoid2.set(Value.kReverse);
    }

    /**
     * Actuate all pistons
     */
    public void actuateAllPistons(){
        actuatePiston1();
        actuatePiston2();
    }

    /**
     * Retract all the pistons
     */
    public void retractAllPistons(){

    }
}