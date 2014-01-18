#include "WPILib.h"
#include <math.h>
#include <HSLImage.h>

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
        Victor *left_1;
        Victor *left_2;
        Victor *right_1;
        Victor *right_2;
		
	// Solenoids
	Solenoid *shiftRight;
	Solenoid *shiftLeft;
	Solenoid *passingPiston;
	
	// Gyro
	Gyro *mainGyro;
	
	// Encoders
	Encoder *leftEncoder;
	Encoder *rightEncoder;
	
	//Axis Camera
	AxisCamera * camera;
		
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
                left_1  = new Victor(1);
                left_2  = new Victor(2);
                right_1 = new Victor(3);
                right_2 = new Victor(4);
                shiftRight = new Solenoid(1);
                shiftLeft  = new Solenoid(2);
                mainGyro = new Gyro(5);
                leftEncoder = new Encoder(6);
                rightencoder = new Encoder(7);
                // Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
                m_robotDrive = new RobotDrive(1, 2, 3, 4);
                gamePad = new Joystick(1);

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
                m_dsPacketsReceivedInCurrentSecond = 0;        // Reset the number of dsPackets in current second
        }

        /********************************** Periodic Routines *************************************/

        void DisabledPeriodic(void) {
        }

        void AutonomousPeriodic(void) 
        {
                initialShot();
                findBall();
                seekAndDestroy();
                reposition();
                shoot();
                findBall();
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
                // shiftRight->Get() : false is low gear, true is high gear
                if(!(shiftRight->Get())) 
                {
                        shiftRight->Set(true);
                        shiftLeft->Set(true);
                }
        }

        void ShiftLow(void) 
        {
                // shiftRight->Get() : false is low gear, true is high gear
                if(shiftRight->Get()) {
                        shiftRight->Set(false);
                        shiftLeft->Set(false);
                }
        }

        void TeleopPeriodic(void)
        {
                bool flag = true; //flag object initial declaration to ensure passing Piston toggle works properly

                float leftStick  = gamePad->GetRawAxis(4);
                float rightStick = gamePad->GetRawAxis(2);
                bool rightBumper = gamePad->GetRawButton(6);
                bool leftBumper  = gamePad->GetRawButton(5);
                bool buttonA     = gamePad->GetRawButton(2);

                if(fabs(leftStick) >= 0.05 || fabs(rightStick >= 0.05)) 
                        {
                                m_robotDrive->TankDrive(leftStick, rightStick);
                        }
                else if(rightBumper || leftBumper) 
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
                                passingPiston->Set(true);
                        }
                 else
                         {
                                if(!buttonA)
                                {
                                        flag = false;
                                        motorControlLeft(0.0);
                                        motorControlRight(0.0);
                                }
                                else
                                {
                                        motorControlLeft(0.0);
                                        motorControlRight(0.0);
                                }
                        }
        } 


	/********************************** Continuous Routines *************************************/
	boolean identifyBall(void)
	{
		//Get axis camera image apply circular identification algorithm
		HSLImage image = new HSLImage(); //should we use HSLImage or RGBImage?
		image = camera -> GetImage(); //gets a new image. check my syntax on this...
		
	}
	void intitalShot(int x)
	{
		while(encoder<x)
		{
		motorControlLeft(1.0);		//move forward for set length determined by encoders position
		motorControlRight(1.0);
		}
		shoot();
	}
	void findBall(void)//1
	{
		while(!identifyBall) //assumes identifyBall returns true if ball is centered
		{
			motorControlLeft(-0.5);
			motorControlRight(0.5);
		}
	}
	void seekAndDestroy(void)//2
	{
		while(identifyBall)
		{
			motorControlLeft(x); //x is the max value for the motors.
			motorControlRight(x);
		}
	}
	void reposition(void)//3
	{
		turn(0); 			// faces the robot in the initial orientation
	}
	void shoot(void)//4
	{
		
	}
	void turn(int x)
	{
		leftEncoder->reset();
		rightEncoder->reset();
		if(int x<=0)
		{
			while(mainGyro>x) ///////encoder check with gyro this code is psuedo for the sake of outline not actual content of the while loop 
								//// Gyro needs to have an accepted angle value. +- 5 degrees? or so most likely less, the accepted angle must be based off the distance to the ball we can discuss this today.
			{
				motorControlLeft(-0.5);
				motorControlRight(0.5);
			}
		}
		else
		{
			while(mainGyro<x) ////////Encoder -> turn radius shit here check with gyro 
			{
				motorControlleft(0.5);
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
