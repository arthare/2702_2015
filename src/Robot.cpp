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
	enum CURDIR
	{
		UNSET,
		FORWARD,
		REVERSE,
	};
public:
	Piston(Solenoid*_fwdsol, Solenoid*_revsol):
		offtime(Timer::GetFPGATimestamp()),
		fwdsol(_fwdsol),
		revsol(_revsol),
		myDir(UNSET)

	{

	}
	void Forward()
	{
		if(fwdsol->Get() || myDir == FORWARD)
		{

		}
		else
		{

			myDir = FORWARD;
			offtime=Timer::GetFPGATimestamp()+.5;
			fwdsol->Set(true);
			revsol->Set(false);
		}
	}
	bool IsForward()
	{
		if(myDir==FORWARD)
		{
			return true;
		}
		else if(myDir==REVERSE)
		{
			return false;
		}
		return false;
	}
	void Reverse()
	{
		if(revsol->Get() || myDir == REVERSE)
		{

		}
		else
		{
			myDir = REVERSE;
			offtime=Timer::GetFPGATimestamp()+.5;
			fwdsol->Set(false);
			revsol->Set(true);

		}
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
	CURDIR myDir;
};
class Pickup
{
public:
	void tick()
	{

	}
	Pickup(Joystick*_stick,RobotDrive*_drive,DigitalInput*_dropoffsensor,SpeedController*_winch,DigitalInput*_bottomsensor,Joystick*_opstick, Gyro* gyro,DigitalInput*_pickuptest)
	:
		mystate(DRIVE),
		stick(_stick),
		drive(_drive),
		dropoffsensor(_dropoffsensor),
		winch(_winch),
		bottomsensor(_bottomsensor),
		opstick(_opstick),
		gyroWrapper(gyro),
		turnController(0.020, 0.00005, 0, &gyroWrapper, &pidReceiver),
		pickuptest(_pickuptest)
	{
		turnController.Enable();
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

	class GyroWrapper : public PIDSource
	{
	public:
		GyroWrapper(Gyro* gyro) : gyro(gyro),last(0) {};
		virtual double PIDGet()
		{
			double now = gyro->GetRate()*0.3 + last*0.7;
			last = now;
			return now;
		}
	private:
		Gyro* gyro;
		double last;
	};
	class PIDReceiver : public PIDOutput
	{
	public:
		virtual void PIDWrite(float output)
		{
			m_output = output;
		}
		double m_output;
	};
	GyroWrapper gyroWrapper;
	PIDReceiver pidReceiver;
	PIDController turnController;
	DigitalInput* pickuptest;
};
class Robot: public OurSampleRobot
{
#ifdef FRC2014
	Compressor compressor;
#endif
	Talon winch;
	Talon stacker;
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
	Solenoid armsforward;
	Solenoid armsreverse;
	Piston armspiston;
#ifndef FRC2014
	DigitalInput leftencA;
	DigitalInput leftencB;
	Encoder leftencoder;
#endif
	DigitalInput dropoffsensor;
	DigitalInput bottomsensor;
	Gyro gyro;
	DigitalInput pickuptest;
	Pickup pickup;

public:
	Robot() :
#ifdef FRC2014
		compressor(PRESSURE_INPUT_DIO, COMPRESSOR_RELAY),
#endif
			winch(WINCH_PWM),
			stacker(STACKER_PWM),
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
			armsforward(OPEN_ARMS_SOLENOID),
			armsreverse(CLOSE_ARMS_SOLENOID),
			armspiston(&armsforward, &armsreverse),
#ifndef FRC2014
			leftencA(LEFT_ENCODER_A_DIO),
			leftencB(LEFT_ENCODER_B_DIO),
			leftencoder(&leftencA, &leftencB),
#endif
			dropoffsensor(DROPOFF_LIMIT_DIO),
			bottomsensor(BOTTOM_LIMIT_DIO),
			gyro(GYRO_ANALOG_CHANNEL),
			pickuptest(PICK_UP_LIMIT_DIO),
			pickup(&stick,&myRobot,&dropoffsensor,&winch,&bottomsensor,&opstick,&gyro,&pickuptest)
	{

		myRobot.SetExpiration(0.1);
#ifdef FRC2014
		compressor.Start();
#endif
	}
	virtual void RobotInit()
	{

#ifdef FRC2014
		rightencoder.Start();
#else
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture(CAMERA_NAME);
#endif
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

	//void TurnTo(double inches, DriveToStopper*stopper)
	//{
		//const double degeesetoturn=90;


	//}

	void DriveTo(double inches, DriveToStopper*stopper)
	{
#ifdef FRC2014
		const double ticksperencrev=250;
		const double wheeldiameter=6;
		const double drivenwheelgearteeth=22;
		const double outputteeth=12;
#else
		const double ticksperencrev=360;
		const double wheeldiameter=6;
		const double drivenwheelgearteeth=24;
		const double outputteeth=15;
#endif


		const double inchesperwheelturn=PI*wheeldiameter;
		const double outputturnsperwheelturn=drivenwheelgearteeth/outputteeth;
		const double ticksperwheelturn=ticksperencrev*outputturnsperwheelturn;
		const double ticksperinch=ticksperwheelturn/inchesperwheelturn;

		const double tickswanted=ticksperinch*inches+rightencoder.GetRaw();

		const double desiredegreese=gyro.GetAngle();

		while(stopper->ShouldContinueDriveTo())
		{
			const double curTicks = rightencoder.GetRaw();
			const double ticksToGo = tickswanted - curTicks;

			double error=desiredegreese - gyro.GetAngle();
			printf("%f %f \n", curTicks, tickswanted);
			//right encoder isn't working
			if(curTicks<tickswanted)
			{
				if(desiredegreese!=gyro.GetAngle())
				{
					this->myRobot.Drive(-0.3,error/40);  //negative means forward
				}
				else
				{
					this->myRobot.Drive(-0.3,0); //negative means forward
				}
			}
			else
			{
				//we've driven far enough
				this->myRobot.Drive(0,0);
				break;
			}
			Wait(0.05);
		}
	}

	void Autonomous()
	{
		myRobot.SetSafetyEnabled(false);
		winch.SetSafetyEnabled(false);
		myRobot.SetExpiration(200);
		winch.SetExpiration(200);


#ifdef FRC2014
		GetWatchdog().SetEnabled(false);
#endif
		int automode=0;
		if(automode==DRIVE_FORWARD)
		{
#ifdef FRC2014
			this->GetWatchdog().Feed();
#endif
			AutonomousDriveToStopper stopper;
			this->DriveTo(DISTANCE_FOR_AUTOMODE_DRIVE_FORWARD,&stopper);
		}

		myRobot.Drive(0,0);
	}

	void DoDriving()
	{
		float winchpower=opstick.GetRawAxis(Joystick::kDefaultYAxis);

		if(stick.GetRawButton(LIFT_ARMS_BUTTON))
		{
			winchpower=1.0;
		}
		else if(stick.GetRawButton(LOWER_ARMS_BUTTON))
		{
			winchpower=-0.65;
		}

		if(stick.GetRawButton(READY_CONTAINER_PICKUP_BUTTON))
		{
			this->armspiston.Forward();
			this->holderpiston.Reverse();
			winchpower=-0.65;
		}
		else if (stick.GetRawButton(LIFT_CONTAINER_BUTTON))
		{
			this->armspiston.Reverse();
			this->holderpiston.Reverse();
			winchpower=0.65;
		}

		winch.Set(winchpower);

		// a test art did revealed that the first 0.3 of motor output does nothing, so let's make the joystick powers start at 0.3...
		float flTurnPower = abs(stick.GetX());
		flTurnPower *= 0.85;
		flTurnPower += 0.15;
		flTurnPower = stick.GetX() > 0 ? flTurnPower : -flTurnPower;
		myRobot.ArcadeDrive(stick.GetY(),stick.GetX());
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
		winch.SetSafetyEnabled(false);
		while (IsOperatorControl() && IsEnabled())
		{
			tiltpiston.Tick();
			holderpiston.Tick();

			//printf("gyro's current reading %f \n",gyro.GetAngle());
#ifdef FRC2014
			this->GetWatchdog().Feed();
#endif
			if(opstick.GetRawButton(OP_STACK_UP))
			{
				stacker.Set(1.0);
			}
			else if(opstick.GetRawButton(OP_STACK_DOWN))
			{
				stacker.Set(-0.5);
			}
			else
			{
				stacker.Set(0);
			}
			DoDriving();

			// handling the tilt piston
			if(stick.GetRawButton(TILT_FORWARD_BUTTON))
			{
				tiltpiston.Forward();
			}
			else
			{
				tiltpiston.Reverse();
			}
			armspiston.Tick();
			if(stick.GetRawButton(OPEN_ARMS_BUTTON))
			{
				armspiston.Forward();
			}
			else if(stick.GetRawButton(CLOSE_ARMS_BUTTON))
			{
					armspiston.Reverse();
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


			{ // arm toggle
				static bool oldopButtons[12] = {0};
				static bool olddriveButtons[12] = {0};
				// check for button-presses here
				if((opstick.GetRawButton(OPSTICK_ARM_TOGGLE) && !oldopButtons[OPSTICK_ARM_TOGGLE]) ||
						(stick.GetRawButton(DRIVE_ARM_TOGGLE) && !olddriveButtons[DRIVE_ARM_TOGGLE]))
				{
					if(armspiston.IsForward())
					{
						armspiston.Reverse();
					}
					else
					{
						armspiston.Forward();
					}
				}


				for(int x = 0;x < 12; x++)
				{
					oldopButtons[x] = opstick.GetRawButton(x);
					olddriveButtons[x] = stick.GetRawButton(x);
				}
			}
			Wait(0.05);
		}
	}

};

START_ROBOT_CLASS(Robot);
