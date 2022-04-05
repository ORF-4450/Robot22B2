
package Team4450.Robot22.subsystems;

import static Team4450.Robot22.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.unmanaged.Unmanaged;

import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import Team4450.Lib.SRXMagneticEncoderRelative.DistanceUnit;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Robot22.RobotContainer;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase 
{
	private WPI_TalonSRX		LFCanTalon, LRCanTalon, RFCanTalon, RRCanTalon;

    private DifferentialDrive	robotDrive;
    
	private DifferentialDriveOdometry	odometer;
  
	// SRX magnetic encoder plugged into a CAN Talon.
	public SRXMagneticEncoderRelative	leftEncoder, rightEncoder;
	
	// Simulation classes help us simulate our robot
	private DifferentialDrivetrainSim 	driveSim;
	private EncoderSim 					leftEncoderSim, rightEncoderSim;
	private Encoder						leftDummyEncoder, rightDummyEncoder;
	private AnalogGyroSim				gyroSim;
	private AnalogGyro					dummyGyro;
	
	// The Field2d class simulates the field in the sim GUI. Note that we can have only one
  	// instance!
  	private Field2d 					fieldSim;

	private ValveDA				highLowValve = new ValveDA(HIGHLOW_VALVE);

	private boolean				talonBrakeMode, lowSpeed, highSpeed;
	
	private double				cumulativeLeftDist = 0, cumulativeRightDist = 0;
	private double				lastLeftDist = 0, lastRightDist = 0;
	
	private SlewRateLimiter		leftLimiter = new SlewRateLimiter(0.5);
	private SlewRateLimiter		rightLimiter = new SlewRateLimiter(0.5);
 	
	 /**
	 * Creates a new DriveBase Subsystem.
	 */
	public DriveBase()
	{
		Util.consoleLog();
		
		// Create the drive Talons.
		LFCanTalon = new WPI_TalonSRX(LF_TALON);
		LRCanTalon = new WPI_TalonSRX(LR_TALON);
		RFCanTalon = new WPI_TalonSRX(RF_TALON);	
		RRCanTalon = new WPI_TalonSRX(RR_TALON);	

		// Initialize CAN Talons and write status to log so we can verify
		// all the Talons are connected.
		InitializeCANTalon(LFCanTalon);
		InitializeCANTalon(LRCanTalon);
		InitializeCANTalon(RFCanTalon);
		InitializeCANTalon(RRCanTalon);
		
		// Configure CAN Talons with appropriate inversion determined by testing.
		// These inversions don't work under simulation due to how inversion works on a real
		// CanTalon. See code in simulationPeriodic for more on inversion handling.
		
		LFCanTalon.setInverted(true);
		LRCanTalon.setInverted(true);
		  
		RFCanTalon.setInverted(false);
		RRCanTalon.setInverted(false);
		  
		// Configure SRX encoders as needed for measuring velocity and distance. 
		// Wheel diameter is in inches. Adjust for each years robot.
		
		rightEncoder = new SRXMagneticEncoderRelative(RRCanTalon, DRIVE_WHEEL_DIAMETER);
		leftEncoder = new SRXMagneticEncoderRelative(LRCanTalon, DRIVE_WHEEL_DIAMETER);
          
        // The real robot has to invert the right encoder so both encoder read increasing
        // values going forward. This should be the same for simulation, but it did not 
        // work right so no invert under sim. I am sure this is due to a mistake in how
        // the simulation is coded, but going to live with it for now.
        //if (RobotBase.isReal()) rightEncoder.setInverted(true);

        // Needed for Built-in SRX encoder sim support, not used at this time.
        if (RobotBase.isSimulation())
        {
            leftEncoder.setInverted(true);
            //rightEncoder.setInverted(true);
        }
		
		// Put rear talons into a differential drive object and set the
	    // front talons to follow the rears.
		  
		LFCanTalon.set(ControlMode.Follower, LRCanTalon.getDeviceID());
		RFCanTalon.set(ControlMode.Follower, RRCanTalon.getDeviceID());

		robotDrive = new DifferentialDrive(LRCanTalon, RRCanTalon);

   		// Configure starting motor safety. This runs a timer between updates of the
		// robotDrive motor power with the set() method. If the timer expires because
		// of no input, the assumption would be that something has done wrong and the
		// code is no longer feeding the robotDrive with speed commands and so the
		// robot could be in an uncontrolled state. So the watchdog turns off the
		// motors until a new input is delivered by the set() method. The problem is
		// with command based scheme, the drive command is executed once per scheduler
		// run and if there are many commands or long running commands, the scheduler
		// may well not make it back to executing the drive command before the timer
		// expires. Experimentally determined 1 sec timer allows enough time for our
		// commands to complete and scheduler returns to the drive command. 1 sec is
		// a long time out but hopefully the robot cannot go too far off the reservation
		// in one second if some problem prevents new set() calls. Conceivably a command
		// that loops in execute would cause the whole scheduler based scheme to stop
		// and so the drive command would not be run and the robot would drive at last
		// power setting until the watchdog shuts the robotDrive down.
		// One other note: as the length of the command list during a scheduler run
		// lengthens or the commands take too much time, the rate at which the Drive
		// command feeds the drive base will slow down and could lead to a lack of
		// driving response. Using the Drive command as the default command of the
		// DriveBase means the joy sticks will be fed to the motors ONCE per scheduler
		// run. Technically the scheduler runs each time RobotPeriodic() is called which
		// is supposed to be every .02 sec. However, the actual time between runs is
		// the total time of all commands executed and then to the next .02 sec call
		// to RobotPeriodic(). Note that the FIRST lower level code also runs a watchdog
		// on each execution of the .02 sec loop and will raise a warning if your
		// code takes more than .02 sec to complete. It may be hard to stay under
		// that time. When it trips, the watchdog will print to the console some somewhat
		// useful information to help determine where the time is being used. This 
        // watchdog timeout cannot be set or turned off. Note it is currently "turned"
        // off by copying the underlying Wpilib code into this project and modifying
        // it to allow control of the watchdog displays. Trying this as under normal
        // conditions the watchdog floods the Riolog with messages making it very 
        // hard to use.
		   
   		robotDrive.stopMotor();
   		robotDrive.setSafetyEnabled(false);	// Will be enabled by the Drive command.
		robotDrive.setExpiration(1.0);
		
		// We do this because we use curvatureDrive in our auto routines for both straight and
		// curved/rotation drivng. We use quickturn feature of curvatureDrive for rotation. If
		// don't set this threshold to zero, curvatureDrive will compute a compensation factor
		// during quickturn which it then applies to next call to curvatureDrive without quickturn.
		// This causes that first call to curvatureDrive without quickturn to compute power values
		// that are incompatible with what we are typically doing, which is to drive straight after
		// a rotation.
	
		//robotDrive.setQuickStopThreshold(0); Removed from 2022 WPILIb. Substitute not yet identified.

		// Always start with braking enabled.
   		
        SetCANTalonBrakeMode(true);
           
        // Always start with voltage compensation enabled for drive talons.

        enableCANTalonVoltageCompenstation(true);
		
		// Always start with gearbox set to low speed.

		lowSpeed();
		
		// Create an odometer object to track the robot's movements.
		
		odometer = new DifferentialDriveOdometry(RobotContainer.navx.getTotalYaw2d());

		// Set robot initial position. This is normally set in an auto routine that
		// starts a match at a particular location and angle. If there is no auto
		// defining an initial position, then pose tracking is difficult because it
		// has to start from a known position. If you start in teleop, you would need
		// to define a fixed starting location or perhaps compute it depending on which
		// driver station location and always start in the same place relative to the
		// station.
		
		resetOdometer(new Pose2d(INITIAL_X, INITIAL_Y, new Rotation2d()), INITIAL_HEADING);

        if (RobotBase.isSimulation()) configureSimulation();
        
        Util.consoleLog("DriveBase created!");
	}

	// Simulation classes help us simulate our robot. Our SRXMagneticEncoderRelative class
	// is not compatible with the Wpilib EncoderSim so create regular Encoder objects which
	// which are compatible with EncoderSim. We then pass the dummy encoders into the SRX
	// encoders which will use the dummy encoders during sim. During sim, the EncoderSim
	// classes drive the dummy encoders which then drive our SRX encoders.
	private void configureSimulation()
	{
		Util.consoleLog();

		// Dummy encoders have to have ports that are not allocated to anything else.
		// Can be non-existent ports.
		leftDummyEncoder = new Encoder(DUMMY_LEFT_ENCODER, DUMMY_LEFT_ENCODER + 1);
        rightDummyEncoder = new Encoder(DUMMY_RIGHT_ENCODER, DUMMY_RIGHT_ENCODER + 1);
		
		double distancePerTickMeters = Math.PI * Units.inchesToMeters(DRIVE_WHEEL_DIAMETER) / SRXMagneticEncoderRelative.TICKS_PER_REVOLUTION;
        
        Util.consoleLog("disttickmeters=%.6f", distancePerTickMeters);
		
		leftDummyEncoder.setDistancePerPulse(distancePerTickMeters);
		rightDummyEncoder.setDistancePerPulse(distancePerTickMeters);

		// Configure our SRXMagneticEncoderRelative instances to use the dummy encoders instead
		// of actual CTRE magnetic encoders connected to TalonSRX controllers.
		//leftEncoder.setSimEncoder(leftDummyEncoder);
        //rightEncoder.setSimEncoder(rightDummyEncoder);
        
        // This code is used with the built-in SRXMagneticEncoder sim support instead of above
        // code. Not used at this time as buit-in sim support is not reliable.
        leftEncoder.initializeSim();
        rightEncoder.initializeSim();

		// Create the encoder simulation classes that wrap the dummy encoders.
		leftEncoderSim = new EncoderSim(leftDummyEncoder);
		rightEncoderSim = new EncoderSim(rightDummyEncoder);	

		// Create the simulation model of the drivetrain.
		// The MOI of 1.0 is guess needed to get the sim to behave like a real robot...
		
		driveSim = new DifferentialDrivetrainSim(
			DCMotor.getCIM(2),       // 2 CIM motors on each side of the drivetrain.
			18.0,                    // 18:1 gearing reduction.
			1.0,                     // MOI of 1.0 kg m^2 (a guess).
			125 * 0.453592,          // The mass (kg) of the robot is approx 60 kg or 125 lbs.
			Units.inchesToMeters(DRIVE_WHEEL_DIAMETER / 2),	// Wheel radius in meters.
			Units.inchesToMeters(TRACK_WIDTH),              // Track width in meters.
			null);

		// Create a dummy analog gyro which will be passed into our NavX wrapper class and will
		// drive that class instead of an actual Navx (See NavX class in RobotLib for more info).
		dummyGyro = new AnalogGyro(SIM_GYRO);

		gyroSim = new AnalogGyroSim(dummyGyro);

		//RobotContainer.navx.setSimGyro(dummyGyro);

        // Use for built-in sim support in NavX instead of above code. Not used at this time as
        // the built-in sim support is not reliable.
        RobotContainer.navx.initializeSim();

		// the Field2d class lets us visualize our robot in the simulation GUI. We have to
		// add it to the dashboard. Field2d is updated by the odometer class instance we
		// use to track robot position.

		fieldSim = new Field2d();
			
		SmartDashboard.putData("Field", fieldSim);
	}
	
	// This method will be called once per scheduler run by the scheduler.
	@Override
	public void periodic() 
	{
		// Update the odometer tracking robot position on the field. We have to track the
		// cumulative encoder counts since at any time we can reset the encoders to facilitate
		// driving functions like auto drive, alt driving mode and more. Odometer wants counts
		// as total since start of match or last odometer reset.		
        
		double left = leftEncoder.getDistance(DistanceUnit.Meters);
		double right = rightEncoder.getDistance(DistanceUnit.Meters);
		
		cumulativeLeftDist += left - lastLeftDist;
		cumulativeRightDist += right - lastRightDist;
		
		lastLeftDist = left;
		lastRightDist = right;
		
    	//Pose2d pose = odometer.update(RobotContainer.navx.getTotalYaw2d(), cumulativeLeftDist, cumulativeRightDist);
    	Pose2d pose = odometer.update(RobotContainer.navx.getTotalAngle2d(), cumulativeLeftDist, cumulativeRightDist);

        if (robot.isEnabled() && RobotBase.isSimulation()) 
        {
            Util.consoleLog();
//			Util.consoleLog("clc=%.3f  crc=%.3f  px=%.3f py=%.3f prot=%.3f tyaw=%.3f", cumulativeLeftDist, cumulativeRightDist,
//							pose.getX(), pose.getY(), pose.getRotation().getDegrees(), RobotContainer.navx.getTotalYaw2d().getDegrees());
            Util.consoleLog("clc=%.3f  crc=%.3f  px=%.3f py=%.3f prot=%.3f tyaw=%.3f", cumulativeLeftDist, cumulativeRightDist,
							pose.getX(), pose.getY(), pose.getRotation().getDegrees(), RobotContainer.navx.getTotalAngle2d().getDegrees());
        }

		// Update the sim field display with the current pose, or position and direction of the robot, after we
		// updated that pose above.
		if (RobotBase.isSimulation()) fieldSim.setRobotPose(pose);
	}
	
    // Updates simulation data *after* to each periodic() (above) call when under simulation. Then all other
    // commands have thier execute funtion called.
	@Override
  	public void simulationPeriodic() 
  	{
		if (robot.isEnabled())
		{
			// To update our simulation, we set motor voltage inputs, update the
			// simulation, and write the simulated positions and velocities to our
			// simulated encoder and gyro.

			//double leftVoltage = LRCanTalon.get() * RobotController.getInputVoltage();
			//double rightVoltage = RRCanTalon.get() * RobotController.getInputVoltage();

			double leftVoltage = LRCanTalon.getMotorOutputVoltage();
			double rightVoltage = RRCanTalon.getMotorOutputVoltage();

			// TalonSRX implemment inversion by switching the output leads of the controller. The simulated
			// controller does not do this, so we have to handle inversion ourselves here.
			
			if (RRCanTalon.getInverted())
			{
				leftVoltage *= -1;
				rightVoltage *= -1;
			}

			driveSim.setInputs(leftVoltage, rightVoltage);
		
			driveSim.update(0.02);

			//Util.consoleLog("linvert=%b  rinvert=%b", LRCanTalon.getInverted(), RRCanTalon.getInverted());

			Util.consoleLog("ltg=%.2f  ltv=%.2f  ldspm=%.4f ldsvms=%.2f", LRCanTalon.get(), leftVoltage,
							driveSim.getLeftPositionMeters(), driveSim.getLeftVelocityMetersPerSecond());
			Util.consoleLog("rtg=%.2f  rtv=%.2f  rdspm=%.4f rdsvms=%.2f", RRCanTalon.get(), rightVoltage,
							driveSim.getRightPositionMeters(), driveSim.getRightVelocityMetersPerSecond());
			
			SmartDashboard.putNumber("LeftDistance", driveSim.getLeftPositionMeters());
			SmartDashboard.putNumber("RightDistance", driveSim.getRightPositionMeters());

			// Drive the dummy encoders (via EncoderSim instances) which in turn drive our SRXMagneticEncoder
			// instances.
			//leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
			//leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());

            // Use with built-in SRX encoder support. Not used at this time.
            leftEncoder.setSimValues(driveSim.getLeftPositionMeters(), driveSim.getLeftVelocityMetersPerSecond());

			//rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
			//rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
			
            // Use with built-in SRX encoder support. Not used at this time.
            rightEncoder.setSimValues(driveSim.getRightPositionMeters(), driveSim.getRightVelocityMetersPerSecond());

            // Update the dummy analog gyro (via GyroSim instance) which drives our NavX class instance.
            // We change the sign because the sign convention of Rotation2d is opposite of our convention used
            // in the Navx class.
		    //gyroSim.setAngle(-driveSim.getHeading().getDegrees());
            
            // Used with built-in NavX sim support. Not used at this time.
            RobotContainer.navx.setSimAngle(-driveSim.getHeading().getDegrees());

			Util.consoleLog("lcount=%d  ldist=%.3fm  lget=%d ldist=%.3fm abs=%.1f", leftDummyEncoder.get(), 
							leftDummyEncoder.getDistance(), leftEncoder.get(), 
							leftEncoder.getDistance(DistanceUnit.Meters),
							leftEncoder.getAbsolute());

			Util.consoleLog("rcount=%d  rdist=%.3fm  rget=%d rdist=%.3fm", rightDummyEncoder.get(), 
							rightDummyEncoder.getDistance(), rightEncoder.get(), 
							rightEncoder.getDistance(DistanceUnit.Meters));

			Util.consoleLog("angle=%.2f  offset=%.2f  dshd=%.3f  hdng=%.2f", dummyGyro.getAngle(), dummyGyro.getOffset(),
                            -driveSim.getHeading().getDegrees(), RobotContainer.navx.getHeading());
                            //RobotContainer.navx.getYawRate());
		}
	}

	/**
	 * Stops all motors.
	 */
	public void stop()
	{
		Util.consoleLog();
		
		robotDrive.stopMotor();
	}
	
	/**
	 * Tank drive function. Passes left/right power values to the robot drive.
	 * Should be called every scheduler run by the Drive command.
	 * @param leftPower Left power setting -1.0 to +1.0.
	 * @param rightPower Right power setting -1.0 to +1.0.
	 * @param squareInputs True reduces sensitivity at low speeds.
	 */
	public void tankDrive(double leftPower, double rightPower, boolean squareInputs)
	{
		robotDrive.tankDrive(leftPower, rightPower, squareInputs);
		
		//Util.consoleLog("l=%.3f m=%.3f  r=%.3f m=%.3f", leftPower, LRCanTalon.get(), rightPower, RRCanTalon.get());
	}
	
	/**
	 * Tank drive function. Passes left/right power values to the robot drive.
	 * Should be called every scheduler run by the Drive command. Uses SlewRateLimiter
	 * filter to modulate inputs to smooth out power delivery. Testing did not show any
	 * advantage over square inputs but will keep this routine for now. In theory, the
     * CAN Talon ramp rate we set globally does the same job.
	 * @param leftSpeed Left power setting -1.0 to +1.0.
	 * @param rightSpeed Right power setting -1.0 to +1.0.
	 */
	public void tankDriveLimited(double leftPower, double rightPower)
	{
		robotDrive.tankDrive(leftLimiter.calculate(leftPower), rightLimiter.calculate(rightPower), false);
		
		//Util.consoleLog("l=%.2f m=%.2f  r=%.2f m=%.2f", leftPower, LRCanTalon.get(), rightPower, RRCanTalon.get());
	}

	/**
	 * Curvature drive function. Drives at set speed with set curve.
	 * @param power Power setting -1.0 to +1.0.
	 * @param rotation Rotation rate -1.0 to +1.0. Clockwise us +.
	 * @param quickTurn True causes quick turn (turn in place).
	 */
	public void curvatureDrive(double power, double rotation, boolean quickTurn)
	{
		robotDrive.curvatureDrive(power, rotation, quickTurn);
 
        // Util.consoleLog("pwr=%.3f rot=%.3f  lget=%.4f(%.4fv)  rget=%.4f(%.4fv)", power, rotation,
        //                 LRCanTalon.get(), LRCanTalon.getMotorOutputVoltage(),
        //                 -RRCanTalon.get(), -RRCanTalon.getMotorOutputVoltage());
    }

    /**
	 * Arcade drive function. Drives at set power with set curve/rotation.
	 * @param power Power setting -1.0 to +1.0, positive is forward.
	 * @param rotation Rotation rate -1.0 to +1.0, positive is clockwise.
	 * @param squareInputs When set reduces sensitivity a low speeds.
	 */
	public void arcadeDrive(double power, double rotation, boolean squareInputs)
	{
		robotDrive.arcadeDrive(power, rotation, squareInputs);
 
        // Util.consoleLog("pwr=%.3f rot=%.3f  lget=%.4f(%.4fv)  rget=%.4f(%.4fv)", power, rotation,
        //                 LRCanTalon.get(), LRCanTalon.getMotorOutputVoltage(),
        //                 -RRCanTalon.get(), -RRCanTalon.getMotorOutputVoltage());
    }
    
    /**
     * Set left/right motor power level directly.
     * @param leftPower     % power -1 to +1, + is forward.
     * @param rightPower    % power -1 to +1, + is forward.
     */
    public void setPower(double leftPower, double rightPower)
    {
        LRCanTalon.set(leftPower);
        RRCanTalon.set(rightPower);
    }
    
    /**
     * Set left/right motor voltage level directly.
     * @param leftVolts     % power -12 to +12, + is forward.
     * @param rightVolts    % power -12 to +12, + is forward.
     */
    public void setVoltage(double leftVolts, double rightVolts)
    {
        LRCanTalon.setVoltage(leftVolts);
        RRCanTalon.setVoltage(rightVolts);
    }
    
    /**
     * It is not clear which of these two methods is correct. The above
     * method should work because we globally reverse motors to make
     * robot drive forward with + power or voltages. However, negation
     * of right side voltage appears to be necessary for trajectory
     * following with voltages. So this method is provided to make
     * the trajectory following Command work. More investigation
     * needed...
	 * Update: With 2022 wpilib, this routine is incorrect. Suspect all
	 * this is related to the built-in inversion of right side in the
	 * wpilib robotdrive class and the confusion it caused us in setting
	 * up the motor inversions. That built-in inversion is gone for 2022.
	 * All this has to be retested for 2022 code migration.
     */
    public void setVoltage2(double leftVolts, double rightVolts)
    {
        setVoltage(leftVolts, -rightVolts);
    }

	// Initialize and Log status indication from CANTalon. If we see an exception
	// or a talon has low voltage value, it did not get recognized by the RR on start up.
	  
	private static void InitializeCANTalon(WPI_TalonSRX talon)
	{
		if (RobotBase.isReal())
			Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

        talon.configFactoryDefault();
		talon.clearStickyFaults(0); //0ms means no blocking.
	}
	  
	/**
	 * Set neutral behavior of drive CAN Talons.
	 * @param brakeMode True = brake mode, false = coast mode.
	 */
	public void SetCANTalonBrakeMode(boolean brakeMode)
	{
		Util.consoleLog("brakes on=%b", brakeMode);
		  
		SmartDashboard.putBoolean("Brakes", brakeMode);

		talonBrakeMode = brakeMode;
		  
		NeutralMode newMode;
		  
		if (brakeMode) 
			newMode = NeutralMode.Brake;
		else 
			newMode = NeutralMode.Coast;
		  
		 LFCanTalon.setNeutralMode(newMode);
		 LRCanTalon.setNeutralMode(newMode);
		 RFCanTalon.setNeutralMode(newMode);
		 RRCanTalon.setNeutralMode(newMode);
	}
	  
	/**
	 * Returns drive Talon brake mode.
	 * @return True if Talons set to brake, false if coast.
	 */
	public boolean isBrakeMode()
	{
		return talonBrakeMode;
	}  
	
	/**
	 * Toggles drive CAN Talon braking mode.
	 */
	public void toggleCANTalonBrakeMode()
	{
		SetCANTalonBrakeMode(!talonBrakeMode);
	}
	  
	/**
	 * Set CAN Talon voltage ramp rate.
	 * @param seconds Number of seconds from zero to full output.
	 * zero disables.
	 */
	public void SetCANTalonRampRate(double seconds)
	{
		Util.consoleLog("%.2f", seconds);
		  
		LFCanTalon.configOpenloopRamp(seconds, 0);
		LRCanTalon.configOpenloopRamp(seconds, 0);
		RFCanTalon.configOpenloopRamp(seconds, 0);
		RRCanTalon.configOpenloopRamp(seconds, 0);
	}
	  
	// Return voltage and current draw for each CAN Talon.
    /**
     * Return voltage and current draw for each CAN Talon.
     * @return Formatted string with the talon information.
     */
	public String GetCANTalonStatus()
	{
		return String.format("%.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f", 
				  LFCanTalon.getMotorOutputVoltage(), LFCanTalon.getStatorCurrent(),
				  LRCanTalon.getMotorOutputVoltage(), LRCanTalon.getStatorCurrent(),
				  RFCanTalon.getMotorOutputVoltage(), RFCanTalon.getStatorCurrent(),
				  RRCanTalon.getMotorOutputVoltage(), RRCanTalon.getStatorCurrent()
				  );
    }
    
    /**
     * Set CAN Talon Voltage Compendation mode on/off. Fixed at 11v max.
     * @param enabled True to turn on voltage compensation, false to turn off.
     */
    public void enableCANTalonVoltageCompenstation(boolean enabled)
    {
        Util.consoleLog("%b", enabled);

        LFCanTalon.configVoltageCompSaturation(11, 0);
        LFCanTalon.enableVoltageCompensation(enabled);
        LRCanTalon.configVoltageCompSaturation(11, 0);
        LRCanTalon.enableVoltageCompensation(enabled);
        RFCanTalon.configVoltageCompSaturation(11, 0);
        RFCanTalon.enableVoltageCompensation(enabled);
        RRCanTalon.configVoltageCompSaturation(11, 0);
        RRCanTalon.enableVoltageCompensation(enabled);
    }

    public void setPowerDeadBand(double power)
    {
        Util.consoleLog("%.2f", power);

        robotDrive.setDeadband(power);
    }
		
	private void updateDS()
	{
		Util.consoleLog("low=%b, high=%b", lowSpeed, highSpeed);
			
		SmartDashboard.putBoolean("Low", lowSpeed);
		SmartDashboard.putBoolean("High", highSpeed);
	}

	/**
	 * Set gear boxes into low speed. Pushes the dog ring to the inside.
	 */
	public void lowSpeed()
	{
		Util.consoleLog();

		highSpeed = false;

		highLowValve.SetA();
			
		lowSpeed = true;
			
		updateDS();
	}

	/**
	 * Set gear boxes into high speed. Pushes the dog ring to the outside.
	 */
	public void highSpeed()
	{
		Util.consoleLog();

		lowSpeed = false;
			
		highLowValve.SetB();
			
		highSpeed = true;
			
		updateDS();
	}

	/**
	 * Return low speed state.
	 * @return True if low speed.
	 */
	public boolean isLowSpeed()
	{
		return lowSpeed;
	}
		
	/**
	 * Return high speed state.
	 * @return True if high speed.
	 */
	public boolean isHighSpeed()
	{
		return highSpeed;
	}
	
	/**
	 * Reset the drive wheel encoders to zero.
	 */
	public void resetEncoders()
	{
		Util.consoleLog();
		
		Util.consoleLog("at encoder reset lget=%d  rget=%d", leftEncoder.get(), rightEncoder.get());
		
		leftEncoder.reset();
		rightEncoder.reset();

		lastLeftDist = lastRightDist = 0;

        RobotContainer.navx.resetYaw();

        // This has the effect of resetting encoder tracking in the driveSim.
		if (driveSim != null) driveSim.setPose(driveSim.getPose());   //odometer.getPoseMeters());
	}
	
	/**
	 * Reset the drive wheel encoders to zero with time delay. There can be a significant delay
	 * between calling for encoder reset and the encoder returning zero. Sometimes this does not
	 * matter and other times it can really mess things up if you reset encoder but at the time
	 * you next read the encoder for a measurement (like in autonomous programs) the encoder has
	 * not yet been reset and returns the previous count. This method resets and delays 112ms
	 * which testing seemed to show would cause the next read of the reset encoder to return
	 * zero. Note, the 112ms delay was with old way of getting encoder counts which had ~100ms
     * response delay plus command send delay. With new way of getting counts, response delay
     * is ~20ms plus ~10ms send delay. So we now wait 35ms to let reset complete.
	 */
	public void resetEncodersWithDelay()
	{
		Util.consoleLog();
		
		Util.consoleLog("at encoder reset lget=%d  rget=%d", leftEncoder.get(), rightEncoder.get());
		
		// Set encoders to update every 20ms just to make sure.
		rightEncoder.setStatusFramePeriod(20);
		leftEncoder.setStatusFramePeriod(20);

		// Reset encoders with 30ms delay before proceeding.
		int rightError = rightEncoder.reset(15);    // 15ms
		int leftError = leftEncoder.reset(15);      // 15ms

		lastLeftDist = lastRightDist = 0;

		if (RobotBase.isSimulation()) driveSim.setPose(driveSim.getPose());   //odometer.getPoseMeters());
		
		Util.consoleLog("after reset lget=%d  rget=%d  lerr=%d  rerr=%d", leftEncoder.get(), rightEncoder.get(),
						leftError, rightError);
		
		if (RobotBase.isSimulation())
			Util.consoleLog("after reset ldget=%d  rdget=%d", leftDummyEncoder.get(), rightDummyEncoder.get());
	}
	
	/**
	 * Return right side motor power.
	 * @return Power level -1.0 to +1.0.
	 */
	public double getRightPower()
	{
		return RRCanTalon.get();
	}
	
	/**
	 * Return left side motor power.
	 * @return Power level -1.0 to +1.0.
	 */
	public double getLeftPower()
	{
		return LRCanTalon.get();
	}
	
	/**
	 * Enable/disable drive base motor safety watchdog.
	 * @param enabled True to enable watchdog, false to disable.
	 */
	public void setMotorSafety(boolean enabled)
	{
		Util.consoleLog("%s", enabled);
		
		robotDrive.setSafetyEnabled(enabled);
	}
	
	/**
	 * Get current pose from odometer. Pose distances in meters.
	 * Pose X is distance along the long side of the field from your driver
	 * station wall. Y is distance along the short side of the field starting
	 * on the left. Angle is cumulative referenced from zero as pointing directly at the
	 * opposition driver station wall, + is left, - is right of that zero
	 * alignment.
	 * @return Current pose.
	 */
	public Pose2d getOdometerPose()
	{
		return odometer.getPoseMeters();
	}
	
	/**
	 * Reset odometer to new position and cumulative angle. Pose x,y distances
	 * in meters, as described in getOdometerPose() doc.
	 * @param pose New starting pose.
	 * @param heading Heading of robot (cumulative angle) in degrees.
	 */
	public Pose2d resetOdometer(Pose2d pose, double heading)
	{
		odometer.resetPosition(pose, Rotation2d.fromDegrees(heading));
		
		Pose2d newPose = odometer.getPoseMeters();

		Util.consoleLog("x=%.2f  y=%.2f  angle=%.2f", newPose.getX(), newPose.getY(), newPose.getRotation().getDegrees());

		if (driveSim != null) driveSim.setPose(newPose);

		cumulativeLeftDist = lastLeftDist = 0;
        cumulativeRightDist = lastRightDist = 0;
        
        return newPose;
    }
    
    /**
     * Zero the odometer pose. Typically used to reset odo during testing to measure
     * distances.
     */
    public void zeroOdometer()
    {
        resetOdometer(new Pose2d(0, 0, new Rotation2d()), 0);
    }
	
	/** 
	 * Average of left and right encoder counts to see how far robot has moved.
	 * @return Average of left & right encoder counts.
	 */
	public int getAvgEncoder()
	{
		return (leftEncoder.get() + rightEncoder.get()) / 2;
	}
	
	/** 
	 * Average of left and right encoder distance to see how far robot has moved.
	 * @return Average of left & right encoder distances (meters).
	 */
	public double getAvgEncoderDist()
	{
		return  (leftEncoder.getDistance(DistanceUnit.Meters) + 
				 rightEncoder.getDistance(DistanceUnit.Meters)) / 2;
	}	
	/** 
	 * Left encoder counts.
	 * @return Left encoder counts.
	 */
	public int getLeftEncoder()
	{
		return leftEncoder.get();
	}
	
	/** 
	 * Right encoder counts.
	 * @return Right encoder counts.
	 */
	public int getRightEncoder()
	{
		return rightEncoder.get();
	}

    /**
     * Returns the turn rate of the robot.
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() 
    {
        return RobotContainer.navx.getYawRate();
    }

    /**
     * Returns the current wheel speeds of the robot.
     * @return The current wheel speeds (meters per second).
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() 
    {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(PIDRateType.velocityMPS), 
                                                rightEncoder.getVelocity(PIDRateType.velocityMPS));
    }

	/**
	 * Displays a trajectory on the sim gui field image.
	 * @param trajectory Trajctory to display.
	 */
	public void displayTrajectoryOnSim(Trajectory trajectory)
	{
		if (RobotBase.isSimulation()) fieldSim.getObject("traj").setTrajectory(trajectory);
	}
}
