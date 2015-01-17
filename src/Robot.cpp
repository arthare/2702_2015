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

class Robot: public OurSampleRobot
{
#ifdef FRC2014
	Compressor compressor;
#endif
	Talon winch;
	RobotDrive myRobot; //Our robot drive system
	Joystick stick; // only joystick
	Joystick opstick;
	Encoder rightencoder;
	Solenoid tiltback;
	Solenoid tiltforward;
	Piston tiltpiston;
	Solenoid closeholder;
	Solenoid openholder;
	Piston holderpiston;


public:
	Robot() :
#ifdef FRC2014
		compressor(PRESSURE_INPUT_DIO, COMPRESSOR_RELAY),
#endif
			winch(WINCH_PWM),
			myRobot(DRIVE_LEFT_FRONT_PWM, DRIVE_LEFT_BACK_PWM,DRIVE_RIGHT_FRONT_PWM, DRIVE_RIGHT_BACK_PWM),	// initialize the RobotDrive to use motor controllers on ports 0 and 1
			stick(DRIVER_JOYSTICK_PORT),
			opstick(OPERATOR_JOYSTICK_PORT),
			rightencoder(RIGHT_ENCODER_A_DIO, RIGHT_ENCODER_B_DIO),
			tiltback(TILT_BACK_SOLENOID),
			tiltforward(TILT_FORWARD_SOLENOID),
			tiltpiston(&tiltforward, &tiltback),
			closeholder(CLOSE_HOLDER_SOLENOID),
			openholder(OPEN_HOLDER_SOLENOID),
			holderpiston(&openholder, &closeholder)

	{
		myRobot.SetExpiration(0.1);
#ifdef FRC2014
		compressor.Start();
#endif
	}

	void DriveTo(double inches)
	{
		const double ticksperencrev=350;
		const double wheeldiameter=6;
		const double drivenwheelgearteeth=24;
		const double outputteeth=8;

		const double inchesperwheelturn=PI*wheeldiameter;
		const double outputturnsperwheelturn=drivenwheelgearteeth/outputteeth;
		const double ticksperwheelturn=ticksperencrev*outputturnsperwheelturn;
		const double ticksperinch=ticksperwheelturn/inchesperwheelturn;
		const double tickswanted=ticksperinch*inches;
		while(true)
		{
			if(rightencoder.GetRaw()<tickswanted)
			{
				this->myRobot.Drive(0.5,0);
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
			winch.Set(opstick.GetRawAxis(Joystick::kDefaultYAxis));
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
		}
	}

};

START_ROBOT_CLASS(Robot);
