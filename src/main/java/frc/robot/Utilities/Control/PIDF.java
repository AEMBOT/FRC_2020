package frc.robot.Utilities.Control;

public class PIDF extends PID{

    // Feedforward value
    double feedForward;

    /**
     * Construct the PID object
     * @param P P gain
     * @param I I gain
     * @param D D gain
     * @param F feed forward gain
     */
    public PIDF(double P, double I, double D, double FF){
        super(P, I, D);
        this.feedForward = FF;
    }

    @Override
    public double calcOutput(double currentValue){
        double feedForwardOutput = (super.getSetpoint() * this.feedForward);

        return super.calcOutput(currentValue) + feedForwardOutput;
    }
}