package Team4450.Robot22.commands;

import Team4450.Lib.Util;
import Team4450.Robot22.subsystems.Pickup;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that manually toggles Pickup operation.
 */
public class PickupDeploy extends CommandBase 
{
  private final Pickup pickup;

  /**
   * Creates a new PickupDeploy command.
   * @param subsystem The subsystem used by this command.
   */
  public PickupDeploy(Pickup subsystem) 
  {
	  Util.consoleLog();
	  
	  pickup = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  addRequirements(pickup);
  }

  /** 
   * Called when the command is initially scheduled.
   * Toggles position of pickup.
   */
  @Override
  public void initialize() 
  {
	  Util.consoleLog();
	  
	  if (pickup.isExtended())
		  pickup.retract();
	  else
		  pickup.extend();
  }

  /**
   *  Returns true when the command should end. This means one execution
   *  accomplished in initialize function.
   */
  @Override
  public boolean isFinished() 
  {
	  return true;
  }
}
