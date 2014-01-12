#include "WPILib.h"
#include <iostream>
#include <math.h>

class BuiltinDefaultCode : public IterativeRobot{
	
	// Declare variable for the robot drive system
	RobotDrive *m_robotDrive;	// robot will use PWM 1-4 for drive motors
	Victor *right_1;
	Victor *right_2;
	Victor *left_1;
	Victor *left_2;
	Joystick *driveController;
	Solenoid *passingPiston;
											// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// driver station object
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
	
												
	Joystick *driveController;					// Declare variables for the two joysticks being used

												// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
		
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
		left_1=new Victor(1);
		left_2=new Victor(2);
		right_1=new Victor(3);
		right_2=new Victor(4);
		passingPiston=new Solenoid(1); //////////////////////////////////////////////////////////////ACTUAL PIN?
												// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
		m_robotDrive = new RobotDrive(left_1,left_2,right_1,right_2);

												// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;

												// Define joysticks being used at USB port #1 
		driveController= new Joystick(1);
	}
	
	
	/********************************** Init Routines *************************************/

	void RobotInit(void) {
												// Actions which would be performed once (and only once) upon initialization of the
												// robot would be put here.
		
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) {
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
		ClearSolenoidLEDsKITT();
												// Move the cursor down a few, since we'll move it back up in periodic.
		printf("\x1b[2B");
	}

	void AutonomousInit(void) {
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
		ClearSolenoidLEDsKITT();
	}

	void TeleopInit(void) {
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
		m_driveMode = UNINITIALIZED_DRIVE;		// Set drive mode to uninitialized
		ClearSolenoidLEDsKITT();
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  {
		static INT32 printSec = (INT32)GetClock() + 1;
		static const INT32 startSec = (INT32)GetClock();


		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;
		
		// while disabled, printout the duration of current disabled mode in seconds
		if (GetClock() > printSec) {
			// Move the cursor back to the previous line and clear it.
			printf("\x1b[1A\x1b[2K");
			printf("Disabled seconds: %d\r\n", printSec - startSec);			
			printSec++;
		}
	}

	void AutonomousPeriodic(void) {
		
		m_autoPeriodicLoops++;

		// generate KITT-style LED display on the solenoids
		SolenoidLEDsKITT( m_autoPeriodicLoops );
				
		/* the below code (if uncommented) would drive the robot forward at half speed
		 * for two seconds.  This code is provided as an example of how to drive the 
		 * robot in autonomous mode, but is not enabled in the default code in order
		 * to prevent an unsuspecting team from having their robot drive autonomously!
		 */
		/* below code commented out for safety
		if (m_autoPeriodicLoops == 1) {
			// When on the first periodic loop in autonomous mode, start driving forwards at half speed
			r_robotDrive->Drive(0.5, 0.0);			// drive forwards at half speed
		}
		if (m_autoPeriodicLoops == (2 * GetLoopsPerSec())) {
			// After 2 seconds, stop the robot 
			r_robotDrive->Drive(0.0, 0.0);			// stop robot
		}
		*/
	}

	
	void TeleopPeriodic(void) {
		bool flag=true;
		while(1){
			float speed=0.3;
			float leftStick=driveController->GetRawAxis(4); 									//get stick values
			float rightStick=driveController->GetRawAxis(2);
			bool flag=true;
			if (driveController->GetRawButton(8) && !driveController->GetRawButton(7)){			//turn right slowly on right bumper press 
				left_1->SetSpeed(speed);
				left_2->SetSpeed(speed);
				right_1->SetSpeed(-1*speed);
				right_2->SetSpeed(-1*speed);
				
			}else if (driveController->GetRawButton(7) && !driveController->GetRawButton(8)){ 	//turn left slowly on left bumper
				left_1->SetSpeed(-1*speed);
				left_2->SetSpeed(-1*speed);
				right_1->SetSpeed(speed);
				right_2->SetSpeed(speed);
			
			}
			else if(driveController->GetRawButton(6)&&flag){ //////////////////////////////////////////////////////////// Check button value
				flag=false; 
				passingPiston->Set(!passingPiston->Get());
				
			}
			else{
				if (fabs(leftStick)>= 0.05 || fabs(rightStick)>= 0.05){
					
					m_robotDrive->TankDrive(leftStick, rightStick);

				}elseif(!driveController->GetRawButton(6)&&flag==false){ //Flag change to assure the piston toggles properly 
					flag=true; 
					
				}else{
					left_1->SetSpeed(0); 
					left_2->SetSpeed(0); 
					right_1->SetSpeed(0); 
					right_2->SetSpeed(0);
				}
			}
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;

		/*
		 * No longer needed since periodic loops are now synchronized with incoming packets.
		if (m_ds->GetPacketNumber() != m_priorPacketNumber) {
		*/
			/* 
			 * Code placed in here will be called only when a new packet of information
			 * has been received by the Driver Station.  Any code which needs new information
			 * from the DS should go in here
			 */
			 
			m_dsPacketsReceivedInCurrentSecond++;					// increment DS packets received
						
			// put Driver Station-dependent code here

			} 
		/*
		}  // if (m_ds->GetPacketNumber()...
		*/

	} // TeleopPeriodic(void)


/********************************** Continuous Routines *************************************/

	/* 
	 * These routines are not used in this demonstration robot
	 *
	 * 
	void DisabledContinuous(void) {
	}

	void AutonomousContinuous(void)	{
	}

	void TeleopContinuous(void) {
	}
	*/
			
};
START_ROBOT_CLASS(BuiltinDefaultCode);