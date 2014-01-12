#include "WPILib.h"
#include <math.h>
class BuiltinDefaultCode : public IterativeRobot
{
	// Declare variable for the robot drive system
	RobotDrive *m_robotDrive;		// robot will use PWM 1-4 for drive motors
	
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// driver station object
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
	
	// Declare variables for the two joysticks being used
	Joystick *gamePad;			// joystick 1 (arcade stick or right tank stick)	
	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
	Victor *left_1;
	Victor *left_2;
	Victor *right_1;
	Victor *right_2;
	
		
public:
/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	BuiltinDefaultCode(void)	{
		printf("BuiltinDefaultCode Constructor Started\n");
		left_1 = new Victor(1);
		left_2=new Victor(2);
		right_1=new Victor(3);
		right_2=new Victor(4);
		// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
		m_robotDrive = new RobotDrive(1, 2, 3, 4);
		gamePad=new Joystick(1);
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
	void MotorControlLeft(float speed){
		left_1->SetSpeed(speed);
		left_2->SetSpeed(speed);
	}
	void MotorControlRight(float speed){
		right_1->SetSpeed(speed);
		right_2->SetSpeed(speed);
	}
	void RobotInit(void) {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) {
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
	}

	void AutonomousInit(void) {
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
	}

	void TeleopInit(void) {
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  {

	}

	void AutonomousPeriodic(void) {
	}

	
	void TeleopPeriodic(void) {
		float leftStick=gamePad->GetRawAxis(4);
		float rightStick=gamePad->GetRawAxis(2);
		if(fabs(leftStick)>=0.05 || fabs(rightStick>=0.05)){
			m_robotDrive->TankDrive(leftStick, rightStick);
		}else{
			left_1->SetSpeed(0);
			left_2->SetSpeed(0);
			right_1->SetSpeed(0);
			right_2->SetSpeed(0);
			
		}
	} // TeleopPeriodic(void)


/********************************** Continuous Routines *************************************/
	void DisabledContinuous(void) {
	}

	void AutonomousContinuous(void)	{
	}

	void TeleopContinuous(void) {
	}

	
/********************************** Miscellaneous Routines *************************************/
	
	/**
	 * Clear KITT-style LED display on the solenoids
	 * 
	 * Clear the solenoid LEDs used for a KITT-style LED display.
	 */	

	/**
	 * Generate KITT-style LED display on the solenoids
	 * 
	 * This method expects to be called during each periodic loop, with the argument being the 
	 * loop number for the current loop.
	 * 
	 * The goal here is to generate a KITT-style LED display.  (See http://en.wikipedia.org/wiki/KITT )
	 * However, since the solenoid module has two scan bars, we can have ours go in opposite directions!
	 * The scan bar is written to have a period of one second with six different positions.
	 */

	/**
	 * Display a given four-bit value in binary on the given solenoid LEDs
	 */

			
};

START_ROBOT_CLASS(BuiltinDefaultCode);
