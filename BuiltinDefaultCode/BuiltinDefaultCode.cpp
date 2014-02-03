#include "WPILib.h"
#include <math.h>
#include "Vision/HSLImage.h"

class BuiltinDefaultCode : public IterativeRobot {
	// Declare variable for the robot drive system
	RobotDrive *m_robotDrive; // Robot will use PWM 1-4 for drive motors
	
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds; // Driver station object
	UINT32 m_priorPacketNumber; // Keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond; // Keep track of the DS packets received in the current second
	
	// Declare variables for the two joysticks being used
	Joystick *gamePad; // Joystick 1 (arcade stick or right tank stick)

	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;

	// Motor controllers
	Talon *left_1;
	Talon *left_2;
	Talon *right_1;
	Talon *right_2;
		
	// Solenoids
	Solenoid *shiftRight;
	Solenoid *shiftLeft;
	Solenoid *passingPiston;
	
	// Gyro
	Gyro *mainGyro;
	
	// Encoders
	Encoder *leftEncoder;
	Encoder *rightEncoder;
	Encoder *shooterEncoder; // Not sure if this is actually needed I think so unless we are using a limit switch 
	
	// Axis Camera
	AxisCamera *camera;
	
	// Limit switches
	DigitalInput *limitSwitch;

public:
	/**
	 * Constructor for this "BuiltinDefaultCode" Class.
	 * 
	 * The constructor creates all of the objects used for the different inputs and outputs of
	 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
	 * providing named objects for each of the robot interfaces. 
	 */
	BuiltinDefaultCode(void) {
		printf("BuiltinDefaultCode Constructor Started\n");
		left_1  = new Talon(1);
		left_2  = new Talon(2);
		right_1 = new Talon(3);
		right_2 = new Talon(4);
		shiftRight = new Solenoid(1);
		shiftLeft  = new Solenoid(2);
		mainGyro = new Gyro(5);
		leftEncoder    = new Encoder(6, 8); // Second int is a placeholder to fix an error with the code (Encoder takes 2 ints)
		rightEncoder   = new Encoder(7, 9); // Same here
		shooterEncoder = new Encoder(10, 11); // Also same here
		
		gamePad = new Joystick(1);
		
		limitSwitch = new DigitalInput(1); // 1 is a placeholder for the Digital Input location

		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;

		printf("BuiltinDefaultCode Constructor Completed\n");
	}
	
	
	/********************************** Init Routines *************************************/


	void RobotInit(void) {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		shiftLeft->Set(false); //low gear
		shiftRight->Set(false); //low gear
		passingPiston->Set(false); //retracted
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) {
		m_disabledPeriodicLoops = 0; // Reset the loop counter for disabled mode
	}

	void AutonomousInit(void) {
		m_autoPeriodicLoops = 0; // Reset the loop counter for autonomous mode
		mainGyro->Reset();       // Assures us gyro starts at 0 degrees
	}

	void TeleopInit(void) {
		m_telePeriodicLoops = 0; // Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0; // Reset the number of dsPackets in current second
	}

	/********************************** Periodic Routines *************************************/

	void DisabledPeriodic(void) {
	}

	void AutonomousPeriodic(void) 
	{
		initialShot(1); // 1 is a temporary value.
		centerRobot();
		seekAndDestroy();
		reposition();
		shoot();
		centerRobot();
		seekAndDestroy();
		reposition();
		shoot();
	}

	/********************************* Teleop methods *****************************************/
	void motorControlLeft(float speed) 
	{
		left_1->SetSpeed(speed);
		left_2->SetSpeed(speed);
	}

	void motorControlRight(float speed)
	{
		right_1->SetSpeed(speed);
		right_2->SetSpeed(speed);
	}

	void ShiftHigh(void) 
	{
		// shiftRight->Get() return value: false is low gear, true is high gear
		if(!(shiftRight->Get())) 
		{
			shiftRight->Set(true);
			shiftLeft->Set(true);
		}
	}

	void ShiftLow(void) 
	{
		// shiftRight->Get() return value: false is low gear, true is high gear
		if(shiftRight->Get()) {
			shiftRight->Set(false);
			shiftLeft->Set(false);
		}
	}

	void TeleopPeriodic(void)
	{
		bool flag = true; // Flag object initial declaration to ensure passing piston toggle works properly

		float leftStick  = gamePad->GetRawAxis(4);
		float rightStick = gamePad->GetRawAxis(2);
		
		bool rightBumper = gamePad->GetRawButton(6);
		bool leftBumper  = gamePad->GetRawButton(5);
		
		bool buttonA     = gamePad->GetRawButton(2);
		
		if(rightBumper || leftBumper) 
		{
			if(rightBumper && !leftBumper) 
			{
				ShiftHigh();
			}
			else if(leftBumper && !rightBumper) 
			{
				ShiftLow();
			}
		}
		else if(buttonA && flag)
		{
			flag = false;
			passingPiston->Set(!(passingPiston->Get()));	// flag reset to false while button a is not held to avoid continuous switching while a button is held
		}
		else if(!buttonA)
		{
				flag = false;
				motorControlLeft(leftStick);
				motorControlRight(rightStick);
		
			}
			
			motorControlLeft(leftStick);
			motorControlRight(rightStick); // motor speed declarations done at the end to ensure watchdog is continually updated.
	} 


	/********************************** Continuous Routines *************************************/
	int identifyBall(void)
	{
		// Get axis camera image apply circular identification algorithm.
		HSLImage* cameraImage = new HSLImage(); // should we use HSLImage or RGBImage?
		cameraImage = camera -> GetImage(); // gets a new image. check my syntax on this.

		return 0; //temp						
	}

	void initialShot(int x)
	{
		while(shooterEncoder->Get() < x)
		{
			motorControlLeft(1.0); // Move forward for set length determined by encoders position
			motorControlRight(1.0);
		}
		shoot();
	}

	bool centerRobot(void) // 1
	{
		while(!identifyBall()) // Assumes identifyBall returns true if ball is centered
		{
			motorControlLeft(-0.5); // This method needs to take an int 0,1,2,3 from the identifyBall method based off the location of the ball
									// Where 0 means the ball is too the left, 1 the ball is centered, and 2 the ball is to the right, 3, the ball is not onscreen
			motorControlRight(0.5);
		}
		return true; // Temporary value
	}

	void seekAndDestroy(void) // 2
	{
		float x = 1.0; // Temporary value
		while(!limitSwitch->Get())    // There will be a limiter switch in the catapault mechanism the robot should check to see if it captured the ball with this.
		{
			if(centerRobot())
			{
				motorControlLeft(x);  // x is the max value for the motors.
				motorControlRight(x); // This method should call the turn method based off of input from the find ball method, adjusting the angle first
									  // Centering the robot on the ball then driving to the ball 
			}
		}
	}

	void reposition(void) // 3
	{
		turn(0); // Faces the robot in the initial orientation
	}

	void shoot(void) // 4
	{
		
	}

	void turn(int x)
	{
		leftEncoder->Reset();
		rightEncoder->Reset();
		if(x <= 0)
		{
			while(mainGyro->GetAngle() > x) // encoder check with gyro this code is psuedo for the sake of outline not actual content of the while loop 
								// Gyro needs to have an accepted angle value. +- 5 degrees? or so most likely less, the accepted angle must be based off the distance to the ball we can discuss this today.
			{
				motorControlLeft(-0.5);
				motorControlRight(0.5);
			}
		}
		else
		{
			while(mainGyro->GetAngle() < x) // Encoder -> turn radius shit here check with gyro
			{
				motorControlLeft(0.5);
				motorControlRight(-0.5);
			}
		}
	}
	
	void DisabledContinuous(void)
	{

	}
        
	void AutonomousContinuous(void)        
	{

	}

	void TeleopContinuous(void)
	{
			
	}
};

START_ROBOT_CLASS(BuiltinDefaultCode);
