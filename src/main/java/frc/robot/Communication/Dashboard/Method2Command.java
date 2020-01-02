package frc.robot.Communication.Dashboard;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Ref:
 * https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
 * Wrapper class to convert a normal method into a command, allows running of
 * methods from the dashboard
 * 
 * @author Will Richards
 */
public class Method2Command extends InstantCommand {

  // Method that we want to run
  private Runnable method;

  /**
   * Get a reference to and initalize the method we want to run
   * 
   * @param method the method we want to run
   */
  public Method2Command(Runnable method) {
    this.method = method;
  }

  // Called when the command is created
  @Override
  public void initialize() {
    method.run();
  }
}
