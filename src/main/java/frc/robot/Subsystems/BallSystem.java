package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;

/**
 * Class used to control the ball system
 * 
 * @author Will Richards
 */
public class BallSystem{


    //Motor controller used to run the front intake
    private CANSparkMax frontIntakeMotor;

    //Reference to the intake
    private Intake intake;

    //Indexer variable
    private Indexer indexer;

    /**
     * Constructor to initilize the intake motors
     */
    public BallSystem(){
        frontIntakeMotor = new CANSparkMax(RobotMap.FrontIntakeMotor, MotorType.kBrushless);

        //New intake object
        intake = new Intake();

        //New indexer object
        indexer = new Indexer();
    }

    public Intake getIntake(){
        return intake;
    }

    public Indexer getIndexer(){
        return indexer;
    }

     /**
     * Nested class used to handle intake controls while still remaining part of the BallSystem
     */
    public class Intake{
        /**
         * Runs the intake
         */
        public void runFrontIntake(){
            frontIntakeMotor.set(0.3);
        }

        /**
         * Stops running the front intake
         */
        public void stopFrontIntake(){
            frontIntakeMotor.set(0);
        }

        /**
         * Sets the values for the intake 
         * @param value
         */
        public void manualIntake(double value) {
            frontIntakeMotor.set(value * -1);
        }
    }

    /**
     * Class used to handle indexing of balls into the shooter
     */
    public class Indexer{

    }
}