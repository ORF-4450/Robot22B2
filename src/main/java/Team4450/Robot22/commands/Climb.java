
package Team4450.Robot22.commands;

import java.util.function.DoubleSupplier;

import static Team4450.Robot22.Constants.*;

import Team4450.Lib.Util;
import Team4450.Robot22.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Climbing command that feeds % power to the Climber winch.
 */
public class Climb extends CommandBase 
{
  private final Climber climber;
  
  private final DoubleSupplier	climbPower;

  /**
   * Creates a new Climb command.
   *
   * @param subsystem The subsystem used by this command.
   * @param climbPower A double supplier of the speed of climb
   * as % power -1.0 to +1.0.
   */
  public Climb(Climber subsystem, DoubleSupplier climbPower) 
  {
	  Util.consoleLog();
	  
	  climber = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  
	  addRequirements(this.climber);
	  
	  this.climbPower = climbPower;
  }

  /**
   *  Called when the command is initially scheduled or rescheduled.
   *  NOTE: This command is set as the default for the climber. That
   *  means it runs as long as no other command that uses the climber
   *  runs. If another command runs, this command will be interrupted 
   *  and then rescheduled when that other command is finished. If the
   *  robot is disabled and then reenabled, this command has been retained
   *  in memory and is rescheduled. This means this class persists in 
   *  memory as long as the robot is turned on. It will be interrupted
   *  and rescheduled as needed. Commands only cease to exist if they
   *  return true when thier isFinished() method is called by the 
   *  scheduler. You have to keep this in mind in terms of what reset
   *  is needed by this command when the robot is reenabled.
   *   
   *  All commands work like this.
   */
  @Override
  public void initialize() 
  {
	  Util.consoleLog();
  }

  /** 
   * Called every time the scheduler runs while the command is scheduled. Passes
   * the climb power value provided by whatever double provider was passed
   * in the constructor to the climber setWinchPower() function. The provider is
   * typically the utility stick Y deflection value but can be any double provider.
   */
  @Override
  public void execute() 
  {
	// Squaring tones down the responsiveness of the winch.
    
    // Since this is the default command for the Climber, it is executed all the time
    // when robot enabled. So we disable any activity when in auto mode.
    
    if (!robot.isAutonomous())
	    climber.setWinchPower(Util.squareInput(climbPower.getAsDouble()));
  }

  /**
   *  Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
	  Util.consoleLog("interrupted=%b", interrupted);
	  
	  climber.stop();
  }

  /**
   *  Returns true when the command should end. Returning false means it never ends.
   */
  @Override
  public boolean isFinished() 
  {
	  return false;
  }
}
