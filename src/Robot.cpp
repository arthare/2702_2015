#include "WPILib.h"
#include "OurSampleRobot.h"
#include "constants.h"
/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Piston
{
public:
	Piston(Solenoid*_fwdsol, Solenoid*_revsol):
		offtime(Timer::GetFPGATimestamp()),
		fwdsol(_fwdsol),
		revsol(_revsol)
	{

	}
	void Forward()
	{
		offtime=Timer::GetFPGATimestamp()+.5;
		fwdsol->Set(true);
		revsol->Set(false);
	}
	void Reverse()
	{
		offtime=Timer::GetFPGATimestamp()+.5;
		fwdsol->Set(false);
		revsol->Set(true);
	}
	void Neutral()
	{
		offtime=Timer::GetFPGATimestamp();
		fwdsol->Set(false);
		revsol->Set(false);
	}
	void Tick()
	{
		if (Timer::GetFPGATimestamp()>offtime)
		{
			Neutral();
		}
	}
private:
	double offtime;
	Solenoid*fwdsol;
	Solenoid*revsol;
};
class Pickup
{
public:
	void tick()
	{
		float winchpower=opstick->GetRawAxis(Joystick::kDefaultYAxis);
		switch(mystate)
		{
		case DRIVE:
			if(stick->GetRawButton(LIFT_BUTTON))
			{
				Setstate(LIFT,"pressed lift button");
			}
			else
			{
				//Driving and winch done by operator control
			}
			break;
		case LIFT:
			if(!stick->GetRawButton(LIFT_BUTTON))
			{
				Setstate(DRIVE, "released lift button");
			}
			else if(dropoffsensor->Get())
			{
				Setstate(GOBACK,"hit dropoff sensor");
			}
			else
			{
				winchpower=-1;
			}
			break;
		case GOBACK:
			if(!stick->GetRawButton(LIFT_BUTTON))
			{
				Setstate(DRIVE,"released lift button");
			}
			else if(bottomsensor->Get())
			{
				Setstate(DRIVE,"hit bottom sensor");
			}else
			{
				winchpower=.5;
			}
			break;
		}
		winch->Set(winchpower);
		drive->ArcadeDrive(stick);
	}
	Pickup(Joystick*_stick,RobotDrive*_drive,DigitalInput*_dropoffsensor,SpeedController*_winch,DigitalInput*_bottomsensor,Joystick*_opstick)
	:
		mystate(DRIVE),
		stick(_stick),
		drive(_drive),
		dropoffsensor(_dropoffsensor),
		winch(_winch),
		bottomsensor(_bottomsensor),
		opstick(_opstick)
	{

	}
private:
	enum PICKSTATE
		{
			DRIVE,
			LIFT,
			GOBACK,
		};
	void Setstate(PICKSTATE newstate, const char* reason)
	{
		printf("Changed state from %d to %d because '%s'\n", mystate, newstate, reason);
		mystate=newstate;
	}
	PICKSTATE mystate;
	Joystick*stick;
	RobotDrive*drive;
	DigitalInput*dropoffsensor;
	SpeedController*winch;
	DigitalInput*bottomsensor;
	Joystick*opstick;
};
class Robot: public OurSampleRobot
{
#ifdef FRC2014
	Compressor compressor;
#endif
	Talon winch;
	RobotDrive myRobot; //Our robot drive system
	Joystick stick; // only joystick
	Joystick opstick;
	DigitalInput rightencA;
	DigitalInput rightencB;
	Encoder rightencoder;
	Solenoid tiltback;
	Solenoid tiltforward;
	Piston tiltpiston;
	Solenoid closeholder;
	Solenoid openholder;
	Piston holderpiston;
	DigitalInput dropoffsensor;
	DigitalInput bottomsensor;
	Pickup pickup;

public:
	Robot() :
#ifdef FRC2014
		compressor(PRESSURE_INPUT_DIO, COMPRESSOR_RELAY),
#endif
			winch(WINCH_PWM),
			myRobot(DRIVE_LEFT_FRONT_PWM, DRIVE_LEFT_BACK_PWM,DRIVE_RIGHT_FRONT_PWM, DRIVE_RIGHT_BACK_PWM),	// initialize the RobotDrive to use motor controllers on ports 0 and 1
			stick(DRIVER_JOYSTICK_PORT),
			opstick(OPERATOR_JOYSTICK_PORT),
			rightencA(RIGHT_ENCODER_A_DIO),
			rightencB(RIGHT_ENCODER_B_DIO),
			rightencoder(rightencA, rightencB, true, Encoder::k1X),
			tiltback(TILT_BACK_SOLENOID),
			tiltforward(TILT_FORWARD_SOLENOID),
			tiltpiston(&tiltforward, &tiltback),
			closeholder(CLOSE_HOLDER_SOLENOID),
			openholder(OPEN_HOLDER_SOLENOID),
			holderpiston(&openholder, &closeholder),
			dropoffsensor(DROPOFF_LIMIT_DIO),
			bottomsensor(BOTTOM_LIMIT_DIO),
			pickup(&stick,&myRobot,&dropoffsensor,&winch,&bottomsensor,&opstick)

	{
		myRobot.SetExpiration(0.1);
#ifdef FRC2014
		compressor.Start();
#endif
	}
	virtual void RobotInit()
	{
		rightencoder.Start();
	}

	class DriveToStopper
	{
	public:
		virtual bool ShouldContinueDriveTo() = 0;
	};
	class OperatorControlDriveToStopper : public DriveToStopper
	{
	public:
		virtual bool ShouldContinueDriveTo()
		{
			DriverStation* ds = DriverStation::GetInstance();
			return ds->IsEnabled() && ds->IsOperatorControl();
		}
	};
	class AutonomousDriveToStopper : public DriveToStopper
	{
	public:
		virtual bool ShouldContinueDriveTo()
		{
			DriverStation* ds = DriverStation::GetInstance();
			return ds->IsEnabled() && ds->IsAutonomous();
		}
	};

	void DriveTo(double inches, DriveToStopper*stopper)
	{
		const double ticksperencrev=250;
		const double wheeldiameter=6;
		const double drivenwheelgearteeth=22;
		const double outputteeth=12;

		const double inchesperwheelturn=PI*wheeldiameter;
		const double outputturnsperwheelturn=drivenwheelgearteeth/outputteeth;
		const double ticksperwheelturn=ticksperencrev*outputturnsperwheelturn;
		const double ticksperinch=ticksperwheelturn/inchesperwheelturn;

		const double tickswanted=ticksperinch*inches+rightencoder.GetRaw();

		while(stopper->ShouldContinueDriveTo())
		{
			const double curTicks = rightencoder.GetRaw();
			const double ticksToGo = tickswanted - curTicks;

			if(curTicks<tickswanted)
			{
				this->myRobot.Drive(-0.3,0); //negitive means forward
			}
			else
			{
				//we've driven far enough
				this->myRobot.Drive(0,0);
				break;
			}
		}
	}
	/**
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl()
	{
		myRobot.SetSafetyEnabled(false);
		winch.SetSafetyEnabled(false);
		myRobot.SetExpiration(100);
		winch.SetExpiration(100);
#ifdef FRC2014
		GetWatchdog().SetEnabled(false);
#endif
		

		while (IsOperatorControl() && IsEnabled())
		{
			tiltpiston.Tick();
			holderpiston.Tick();

#ifdef FRC2014
			this->GetWatchdog().Feed();

#endif
			pickup.tick();
			// handling the tilt piston
			if(stick.GetRawButton(TILT_BACK_BUTTON))
			{
				tiltpiston.Reverse();
			}
			else if(stick.GetRawButton(TILT_FORWARD_BUTTON))
			{
				tiltpiston.Forward();
			}

			// Handling the crate holder
			if (stick.GetRawButton(OPEN_HOLDER_BUTTON))
			{
				holderpiston.Reverse();
			}
			else if (stick.GetRawButton(CLOSE_HOLDER_BUTTON))
			{
				holderpiston.Forward();
			}
			if(stick.GetRawButton(DRIVE_TO_BUTTON))
			{
				OperatorControlDriveToStopper stopper;
				this->DriveTo(60,&stopper);
				//testing driveto
			}
		}
	}

};

START_ROBOT_CLASS(Robot);
