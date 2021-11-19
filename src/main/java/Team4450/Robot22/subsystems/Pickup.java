package Team4450.Robot22.subsystems;

import static Team4450.Robot22.Constants.*;

import java.util.function.BiConsumer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pickup extends SubsystemBase
{
	private WPI_TalonSRX	pickupTalon;

	private ValveDA			pickupValve = new ValveDA(PICKUP_VALVE);

	private DigitalInput	ballEye = new DigitalInput(BALL_EYE);

	private AsynchronousInterrupt	interruptHandler;

    private Channel         channel;
    private double          pickupPower = .35, interruptTime;
	private boolean			extended = false, pickupRunning = false, interrupted;
    public static boolean   balleye = false;
    
	public Pickup (Channel channel)
	{
		Util.consoleLog();

		pickupTalon = new WPI_TalonSRX(PICKUP_TALON);
		  
		pickupTalon.setInverted(true);

		InitializeCANTalon(pickupTalon);
        
        this.channel = channel;

        // Configure interrupt handler for the ballEye optical ball detector. An interrupt
        // handler will run the code (function) we specifiy when the RoboRio detects a change
        // in the digital signal from the eye.
		
		interruptHandler = new AsynchronousInterrupt(ballEye, handleInterrupt);
		
        // Listen for a falling edge interrupt. This is because "edge" refers to voltage
        // signal returned by the eye to the digital IO class. When the eye is not blocked
        // it returns 5v which the DIO returns as True. We will invert this for our
        // use as we like to think of not blocked as False and blocked as True. 
        // Since we are interested in the transition between not blocked (5v) and
        // blocked (0v), we interrupt on the falling (voltage) edge of the signal.
		
		interruptHandler.setInterruptEdges(false, true);

		retract();
		
		Util.consoleLog("Pickup created!");
    }
    
    // Called on each run of the scheduler.
    @Override
    public void periodic() 
    {
        // So the ball eye started raising double interrupts for single break of
        // the light beam. Could not figure out why so added some code to record
        // the time of an interrupt and wait 1/4 second before responding to a
        // new interrupt.
        if (Util.getElaspedTime(interruptTime) > .25) interrupted = false;
    }

	private void updateDS()
	{
		SmartDashboard.putBoolean("Pickup", pickupRunning);
		SmartDashboard.putBoolean("PickupExtended", extended);
	}
	
	/**
	 * Extend the pickup arm and start the wheel motor with default power.
	 * Method is thread safe.
	 */
	public void extend()
	{
		Util.consoleLog();
		
		synchronized (this)
		{
			pickupValve.SetA();
			
            extended = true;
            
            //channel.startBelt();
			
			start(pickupPower);
		}
	}
	
	/**
	 * Retract the pickup arm and stop the wheel motor.
	 * Method is thread safe.
	 */
	public void retract()
	{
		Util.consoleLog();
		
		synchronized (this)
		{
			pickupValve.SetB();
			
			extended = false;
            
            channel.stopBelt();

			stop();
		}
	}
	  
	/**
	 * Toggle between pickup arm extended and retracted.
	 */
	public void toggleDeploy()
	{
		Util.consoleLog("%b", isExtended());
		
		if (isExtended())
			retract();
		else
		  	extend();
    }
	
	/**
	 * Start pick up wheel and enable optical sensor interrupts.
	 * @param power % power to run wheel motor 0.0->1.0.
	 */
	private void start(double power)
	{
		Util.consoleLog("%.2f", power);
		
		pickupTalon.set(power);
		
		pickupRunning = true;
		
		interruptHandler.enable();
		
		updateDS();
	}

	/**
	 * Stop wheel motor and disable interrupts.
	 */
	private void stop()
	{
		Util.consoleLog();
		
		pickupTalon.stopMotor();
	
		pickupRunning = false;
		
		// Note, the following function is expensive in terms of the
		// time it takes and will trigger the global watchdog warning
		// and the drivebase motor safety will trigger if set below 1
		// second. No idea why this function takes so long.
		
		interruptHandler.disable();
	
		updateDS();
	}
	
	/**
	 * Returns extended state of pickup arm.
	 * @return True if arm extended.
	 */
	public boolean isExtended()
	{
		return extended;
	}
	
	/**
	 * Returns state of wheel motor.
	 * @return True if running.
	 */
	public boolean isRunning()
	{
		return pickupRunning;
	}
		
	// Consumer object to handle detection of ball by the ball Eye
	// optical sensor. The sensor generates a hardware interrupt when the 
	// eye is triggered and the handleInterrupt method is called when the
	// interrupt occurs. Keep the length of the code in that method short 
	// as no new interrupts will be reported until handleInterrupt ends.
	// Note we use a delay scheme to ignore interrupts happening in quick
	// succession, as happens with our mechanisim.

	BiConsumer<Boolean, Boolean > handleInterrupt = (rising, falling) ->
        {
             if (interrupted) return;

	    	 Util.consoleLog("ball interrupt(%b,%b)", rising, falling);

             channel.startBelt();

             Timer.delay(1.5);
             
             channel.stopBelt();

             interrupted = true;
             interruptTime = Util.timeStamp();

	    	 //Channel channel = (Channel) param;
             //channel.intakeBall();    
		};
	
	// private class InterruptHandler extends InterruptHandlerFunction<Object> 
	// {
	//      @Override
	//      public void interruptFired(int interruptAssertedMask, Object param) 
	//      {
    //          if (interrupted) return;

	//     	 Util.consoleLog("ball  interrupt");

    //          channel.startBelt();

    //          Timer.delay(1.5);
             
    //          channel.stopBelt();

    //          interrupted = true;
    //          interruptTime = Util.timeStamp();

	//     	 //Channel channel = (Channel) param;
    //          //channel.intakeBall();
	//      }
	     
//		 public Channel overridableParamter()
//	     {
//			return channel;
//	     }
//	}
	
	/**
	 * Returns state of ball detector electric eye. The electronics returns a high voltage
     * signal (true) when the eye is NOT blocked. But that is inverted from the way we humans 
     * think of this. We expect true when the eye is blocked. So we invert the state of the 
     * eye to match the way humans think of the use of the eye.
     * @return True means ball blocking eye. Will only be true when ball passing eye.
	 */
	public boolean getBallEye()
	{
		return !ballEye.get();
	}

	// Initialize and Log status indication from CANTalon. If we see an exception
	// or a talon has low voltage value, it did not get recognized by the RR on start up.
	  
	private static void InitializeCANTalon(WPI_TalonSRX talon)
	{
		Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

		talon.clearStickyFaults(0); //0ms means no blocking.
	}
}
