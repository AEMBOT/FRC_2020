package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;
import frc.robot.Hardware.Pneumatics.AdvancedCompressor;
import frc.robot.Hardware.Pneumatics.DoublePiston;
import frc.robot.Hardware.Pneumatics.PistonGroup;

/**
 * Class used to control the ball system
 * 
 * @author Will Richards
 */
public class BallSystem{


    //Motor controller used to run the front intake
    private CANSparkMax frontIntakeMotor;

    // Motors to control the indexer wheels
    private CANSparkMax frontIndexerMotor;
    private CANSparkMax backIndexerMotor;

    private CANSparkMax transportBeltMotor;

    //Reference to the intake
    private Intake intake;

    //Indexer variable
    private Indexer indexer;

    //Hopper variable
    private Hopper hopper;

    // Intake pistons
    private DoublePiston intakePiston;

    /**
     * Constructor to initialize the intake motors
     */
    public BallSystem(){
        frontIntakeMotor = new CANSparkMax(RobotMap.FrontIntakeMotor, MotorType.kBrushless);

        frontIntakeMotor.setOpenLoopRampRate(0.5);

        //Indexer motor creation
        backIndexerMotor = new CANSparkMax(RobotMap.BackIndexerMotor, MotorType.kBrushless);
        frontIndexerMotor = new CANSparkMax(RobotMap.FrontIndexerMotor, MotorType.kBrushless);

        backIndexerMotor.follow(frontIndexerMotor);

        transportBeltMotor = new CANSparkMax(RobotMap.BeltMotor, MotorType.kBrushless);

        //New intake object
        intake = new Intake();

        //New indexer object
        indexer = new Indexer();

        //New hopper object
        hopper = new Hopper();

        // Construct Pneumatics
        intakePiston = new DoublePiston(RobotMap.IntakePistonA, RobotMap.IntakePistonB);

        intake.retractIntake();
    }

    /**
     * Get a reference to he Intake sub class
     */
    public Intake getIntake(){
        return intake;
    }

    /**
     * Get a reference to the indexer sub class
     */
    public Indexer getIndexer(){
        return indexer;
    }

    /**
     * Get a reference to the hopper sub class
     */
    public Hopper getHopper() {
        return hopper;
    }

     /**
     * Nested class used to handle intake controls while still remaining part of the BallSystem
     */
    public class Intake{

        private boolean intakeStatus = false;

        /**
         * Runs the intake
         */
        public void runFrontIntakeForward(){
            frontIntakeMotor.set(-0.4);
        }

         /**
         * Runs the intake
         */
        public void runFrontIntakeBack(){
            frontIntakeMotor.set(0.5);
        }

        private void enableIntake(){
            intakeStatus = true;
        }

        private void disableIntake(){
            intakeStatus = false;
        }

        private void runIntake(){
            if(intakeStatus)
                frontIntakeMotor.set(-0.5);
        }

        /**
         * Stops running the front intake
         */
        public void stopFrontIntake(){
            frontIntakeMotor.set(0);
        }

        public boolean isIntakeRunning(){
            return (frontIntakeMotor.get() > 0);
        }

        public double getIntakeCurrent(){
            return frontIntakeMotor.getOutputCurrent();
        }

        /**
         * Sets the values for the intake 
         * @param value
         */
        public void manualIntake(double value) {
            frontIntakeMotor.set(value * -1);
        }

        /**
         * Actuates both pistons causing the intake to extend
         */
        public void extendIntake(){
            intakePiston.actuate();

            // Attempt to maintain a constant pressure by recharging after use
            AdvancedCompressor.startTimedRecharge(0.75);
        }

        /**
         * Retracts both pistons causing the intake to retract
         */
        public void retractIntake(){
            intakePiston.retract();

            // Attempt to maintain a constant pressure by recharging after use
            AdvancedCompressor.startTimedRecharge(0.75);
        }
    }

    /**
     * Class used to handle indexing of balls into the shooter
     */
    public class Indexer{
        /**
         * Run normal shooter index
         */
        public void standardIndex(){
            frontIndexerMotor.set(-1);
            runBelts();
        }

        public void runBelts(){
            transportBeltMotor.set(-0.5);
        }

        public void stopBelts(){
            transportBeltMotor.set(0);
        }

        public double getBeltCurrent(){
            return transportBeltMotor.getOutputCurrent();
        }
        
        public double getFrontIndexerCurrent(){
            return frontIndexerMotor.getOutputCurrent();
        }

        public double getBackIndexerCurrent(){
            return backIndexerMotor.getOutputCurrent();
        }

        /**
         * Stop running the indexer motors
         */
        public void stopIndexing(){
            backIndexerMotor.set(0);
            frontIndexerMotor.set(0);
            stopBelts();
        }
    }

    /**
     * Class used to handle storage and passing of balls to the indexer
     */
    public class Hopper{

    }
}