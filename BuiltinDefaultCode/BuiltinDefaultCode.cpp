#include "WPILib.h"
#include <math.h>
#include "Vision/HSLImage.h"
#define X_IMAGE_RES 320		// X Image resolution in pixels, should be 160, 320 or 640
#define VIEW_ANGLE 48		// Axis 206 camera
//#define VIEW_ANGLE 43.5  // Axis M1011 camera
#define PI 3.141592653

// Score limits used for target identification
#define RECTANGULARITY_LIMIT 60
#define ASPECT_RATIO_LIMIT 75
#define X_EDGE_LIMIT 40
#define Y_EDGE_LIMIT 60

// Minimum area of particles to be considered
#define AREA_MINIMUM 500

// Edge profile constants used for hollowness score calculation
#define XMAXSIZE 24
#define XMINSIZE 24
#define YMAXSIZE 24
#define YMINSIZE 48
const double xMax[XMAXSIZE] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
const double xMin[XMINSIZE] = {.4, .6, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, 0.6, 0};
const double yMax[YMAXSIZE] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
const double yMin[YMINSIZE] = {.4, .6, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
								.05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
								.05, .05, .6, 0};
const double RectHotGoalRatio = 0.95;

class BuiltinDefaultCode : public IterativeRobot {


	// Motor controllers
	Talon *left_1;
	Talon *left_2;
	Talon *right_1;
	Talon *right_2;
	Victor *zoidberg_roller;
	Victor *zoidberg_position;
	Victor *shooter;
	
	// Compressor
	Compressor *m_compressor;	
	
	// Solenoids
	Solenoid *shifter;
	Solenoid *passingPiston;
	
	// Gyro
	Gyro *mainGyro;
	
	// Encoders
	Encoder *leftEncoder;
	Encoder *rightEncoder;

	AnalogChannel *zoidbergAnalog;
	AnalogChannel *distanceSensor;
	
	// Axis Camera
	AxisCamera *camera;

	// Limit switches
	DigitalInput *limitSwitchShooter;
	
	// Joystick
	Joystick *gamePad;
	
	// Declare (x,y) coordinates of the robot on the field.
	double m_x;
	double m_y;

    // Declare angle of the robot.
    double angle;

public:
	BuiltinDefaultCode(void) {
		printf("BuiltinDefaultCode Constructor Started\n");
		
		left_1  = new Talon(1);
		left_2  = new Talon(2);
		right_1 = new Talon(3);
		right_2 = new Talon(4);
		zoidberg_roller = new Victor(8);
		zoidberg_position = new Victor(5);
		shooter = new Victor(7);
		
		// Compressor
		m_compressor = new Compressor(3,2);
		
		// End Compressor
		shifter = new Solenoid(1, 2);
				
		leftEncoder    = new Encoder(4, 5);    // Second int is a placeholder to fix an error with the code (Encoder takes 2 ints)
		rightEncoder   = new Encoder(1, 2);    // Same here
		zoidbergAnalog = new AnalogChannel(2); // potentiometer?
		distanceSensor = new AnalogChannel(1);

		gamePad = new Joystick(1);
		
		limitSwitchShooter = new DigitalInput(6); // 1 is a placeholder for the Digital Input location limit Switch placed on shooter mechanism

		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		// UINT32 m_priorPacketNumber;	// keep track of the most recent packet number from the DS
		// UINT8 m_dsPacketsReceivedInCurrentSecond; // keep track of the ds packets received in the current second

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		// m_autoPeriodicLoops = 0;
		// m_disabledPeriodicLoops = 0;
		// m_telePeriodicLoops = 0;
		
		m_x = 0;
		m_y = 0;

        // Set feet per pulse for encoder.
        leftEncoder->SetDistancePerPulse(.00460243323); //Not the actual value. Find the gear ratio.
        rightEncoder->SetDistancePerPulse(.00460243323);

		printf("BuiltinDefaultCode Constructor Completed\n");
	}
	
	
	/********************************** Init Routines *************************************/


	void RobotInit(void) {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		shifter->Set(false); // low gear
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) {
		// int m_disabledPeriodicLoops = 0; // Reset the loop counter for disabled mode
	}

	void AutonomousInit(void) {
		// int m_autoPeriodicLoops = 0; // Reset the loop counter for autonomous mode
		m_compressor->Start();
	}

	void TeleopInit(void) {

		// int m_telePeriodicLoops = 0; // Reset the loop counter for teleop mode
		// int m_dsPacketsReceivedInCurrentSecond = 0; // Reset the number of dsPackets in current second
		m_compressor->Start();
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

	void moveGobbler(int position) { // 0: 0 degrees 1: 45 degrees 2: 90 degrees?  |_ arm like this, -> is ball 90 degrees, vertical is 0 + is down - is up
		float speed = 0.25;
		if(position == 0) { // might also be smart to call position as a function of the distance sensor, but only one case, like hey if you sense a ball lower it? or 
			// only lower it when there is a ball, but what if we run into another robot, maybe take a button, so like if button B&&ballSense then go to position 2?
			while(zoidbergAnalog->GetValue() >= 0) {
				zoidberg_position->SetSpeed(-speed);
				shooter->Set(0.3);
				left_1 -> Set(0.0);
				left_2 -> Set(0.0);
				right_1 -> Set(0.0);
				right_2 -> Set(0.0);
				zoidberg_roller -> Set(0.0);
			}
		}
		if(position == 1) {
			if(zoidbergAnalog->GetValue() < 45) {
				while(zoidbergAnalog->GetValue() <= 45) {
					zoidberg_position->SetSpeed(speed);
					shooter->Set(0.3);
					left_1 -> Set(0.0);
					left_2 -> Set(0.0);
					right_1 -> Set(0.0);
					right_2 -> Set(0.0);
					zoidberg_roller -> Set(0.0);
				}
			} else if(zoidbergAnalog->GetValue() >= 45) {
				while(zoidbergAnalog->GetValue() >= 45) {
					zoidberg_position->SetSpeed(speed);
					shooter->Set(0.3);
					left_1 -> Set(0.0);
					left_2 -> Set(0.0);
					right_1 -> Set(0.0);
					right_2 -> Set(0.0);
					zoidberg_roller -> Set(0.0);
				}
			}
		} else if(position == 2){
			while(zoidbergAnalog->GetValue() <= 90){
				zoidberg_position->SetSpeed(speed);
				shooter->Set(0.3);
				left_1 -> Set(0.0);
				left_2 -> Set(0.0);
				right_1 -> Set(0.0);
				right_2 -> Set(0.0);
				zoidberg_roller -> Set(0.0);
			}
		}
	}

	void ShiftHigh(void) {
		// shifter->Get() return value: false is low gear, true is high gear
		if(!(shifter->Get())) {
			shifter->Set(true);
			
		}
	}

	void ShiftLow(void) {
		// shifter->Get() return value: false is low gear, true is high gear
		if(shifter->Get()) {
			shifter->Set(false);
		}
	}

	void TeleopPeriodic(void) {
		m_compressor->Start();
		bool flag = true; // Flag object initial declaration to ensure passing piston toggle works properly

		float leftStick  = gamePad->GetRawAxis(2);
		float rightStick = gamePad->GetRawAxis(4);
				
		bool rightBumper = gamePad->GetRawButton(6);
		bool leftBumper  = gamePad->GetRawButton(5);
		
		bool buttonA     = gamePad->GetRawButton(2);
		// bool buttonB     = gamePad->GetRawButton(3);
		
		if(rightBumper || leftBumper) {
			if(rightBumper && !leftBumper) {
				ShiftHigh();
			} else if(leftBumper && !rightBumper) {
				ShiftLow();
			}
		} else if(buttonA) {
			shoot();
			flag = false;
		} else if(!buttonA) {
			flag = true;
			motorControlLeft(-1 * leftStick);
			motorControlRight(rightStick);
		}
		
		// Pickup sequence:
		// 
		// User presses a button
		// Zoidberg roller spins up, lowered into a position defined by potentiometer
		// Once reached, it stops lowered
		// If grabber down and button pressed or distance sensor exceeds a certain threshold (moving average) for about 0.5 seconds
		// (Above configurable at the top of the file)
		// Moves back up to previous position while still spinning
		// Once in position, roller stops.
		
		// Third position for gobbler between 0 degrees and pickup position when firing
		
		// 

		motorControlLeft(-1 * leftStick);
		motorControlRight(rightStick); // motor speed declarations done at the end to ensure watchdog is continually updated.

		shooter->SetSpeed(0.0);

		// zoidberg_roller->SetSpeed(1.0);
		// zoidberg_roller -> Set(pass(buttonB));
	} 

	/********************************** Continuous Routines *************************************/

	int identifyBall(void) {

//		Threshold threshold(60, 100, 90, 255, 20, 255);	//HSV threshold criteria, ranges are in that order ie. Hue is 60-100
//		ParticleFilterCriteria2 criteria[] = {
//				{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}
//		}; //Particle filter criteria, used to filter out small particles
		// AxisCamera &camera = AxisCamera::GetInstance();	//To use the Axis camera uncomment this line
		
		/**
		 * Do the image capture with the camera and apply the algorithm dscribed above. This
		 * sample will either get images from the camera or from an image file stored in the top
		 * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
		 */

		// ColorImage *image;
		//image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash

		// camera->GetImage(image);				//To get the images from the camera comment the line above and uncomment this one
		// BinaryImage *thresholdImage = image->ThresholdHSV(threshold);	// get just the green target pixels
		//thresholdImage->Write("/threshold.bmp");
		// BinaryImage *convexHullImage = thresholdImage->ConvexHull(false);  // fill in partial and full rectangles
		//convexHullImage->Write("/ConvexHull.bmp");
		// BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 1);	//Remove small particles
		//filteredImage->Write("Filtered.bmp");

		// vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  //get a particle analysis report for each particle
		// scores = new Scores[reports->size()];
		
		//Iterate through each particle, scoring it and determining whether it is a target or not
//		for (unsigned i = 0; i < reports->size(); i++) {
//			ParticleAnalysisReport *report = &(reports->at(i));
			
//				scores[i].rectangularity = scoreRectangularity(report);
//				scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage, report, true);
//				scores[i].aspectRatioInner = scoreAspectRatio(filteredImage, report, false);			
//				scores[i].xEdge = scoreXEdge(thresholdImage, report);
//				scores[i].yEdge = scoreYEdge(thresholdImage, report);
			
//				if(scoreCompare(scores[i], false))
//				{
//					printf("particle: %d  is a High Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
//					printf("Distance: %f \n", computeDistance(thresholdImage, report, false));
//				} else if (scoreCompare(scores[i], true)) {
//					printf("particle: %d  is a Middle Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
//					printf("Distance: %f \n", computeDistance(thresholdImage, report, true));
//				} else {
//					printf("particle: %d  is not a goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
//				}
			// printf("rect: %f  ARinner: %f \n", scores[i].rectangularity, scores[i].aspectRatioInner);
			// printf("ARouter: %f  xEdge: %f  yEdge: %f  \n", scores[i].aspectRatioOuter, scores[i].xEdge, scores[i].yEdge);	
//		}
//		printf("\n");
//		
//		// be sure to delete images after using them
//		delete filteredImage;
//		delete convexHullImage;
//		delete thresholdImage;
//		delete image;
//		
//		//delete allocated reports and Scores objects also
//		// delete scores;
//		delete reports;
		return 0;
	}
	/*
	double computeDistance (BinaryImage *image, ParticleAnalysisReport *report, bool outer) {
		double rectShort, height;
		int targetHeight;
	
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
		//using the smaller of the estimated rectangle short side and the bounding rectangle height results in better performance
		//on skewed rectangles
		height = min(report->boundingRect.height, rectShort);
		targetHeight = outer ? 29 : 21;
	
		return X_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
	}

	double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool outer){
		double rectLong, rectShort, idealAspectRatio, aspectRatio;
		idealAspectRatio = outer ? (62/29) : (62/20);	//Dimensions of goal opening + 4 inches on all 4 sides for reflective tape
	
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
	
		//Divide width by height to measure aspect ratio
		if(report->boundingRect.width > report->boundingRect.height){
			//particle is wider than it is tall, divide long by short
			aspectRatio = 100*(1-fabs((1-((rectLong/rectShort)/idealAspectRatio))));
		} else {
			//particle is taller than it is wide, divide short by long
			aspectRatio = 100*(1-fabs((1-((rectShort/rectLong)/idealAspectRatio))));
		}
		return (max(0, min(aspectRatio, 100)));		//force to be in range 0-100
	}
	*/

	void initialShot(int x) {
//		while(shooterEncoder->Get() < x)
//		{
//			motorControlLeft(1.0); // Move forward for set length determined by encoders position
//			motorControlRight(1.0);
//		}
//		shoot();
	}

	bool centerRobot(void) {
		while(!identifyBall()) // Assumes identifyBall returns true if ball is centered
		{
			motorControlLeft(-0.5); // This method needs to take an int 0,1,2,3 from the identifyBall method based off the location of the ball
									// Where 0 means the ball is too the left, 1 the ball is centered, and 2 the ball is to the right, 3, the ball is not onscreen
			motorControlRight(0.5);
		}
		return true; // Temporary value
	}

	void seekAndDestroy(void) {
		float x = 1.0; // Temporary value
		while(!limitSwitchShooter->Get()) { // There will be a limit switch in the catapault mechanism the robot should check to see if it captured the ball with this.
			if(centerRobot()) {
				motorControlLeft(x);  // x is the max value for the motors.
				motorControlRight(x); // This method should call the turn method based off of input from the find ball method, adjusting the angle first
									  // Centering the robot on the ball then driving to the ball 
			}
		}
	}

	void reposition(void) {
		//turn(0); // Faces the robot in the initial orientation
	}

	void shoot(void) {
		moveGobbler(0);
		// Not sure about the below values
		while(!limitSwitchShooter->Get()) { // Limit switch: false = pressed, true = not pressed
			shooter->Set(0.3);
			left_1 -> Set(0.0);
			left_2 -> Set(0.0);
			right_1 -> Set(0.0);
			right_2 -> Set(0.0);
			zoidberg_roller -> Set(0.0);
			zoidberg_position -> Set(0.0);
		}

		while(limitSwitchShooter->Get()) {
			shooter->Set(0.5);
			left_1 -> Set(0.0);
			left_2 -> Set(0.0);
			right_1 -> Set(0.0);
			right_2 -> Set(0.0);
			zoidberg_roller -> Set(0.0);
			zoidberg_position -> Set(0.0);
		}

		shooter->Set(0);
		moveGobbler(2);
	}

	float pass(bool buttonB) {
		float speed = 0.25;
		moveGobbler(2); //Someone write this method!
		if (buttonB)
		{
			return speed;
		}
		else
		{
			return 0.0;
		}
	}

	void turn(int x) { // x is an angle in radians. Left turn is positive. Right turn is negative. 
		leftEncoder->Reset();
		rightEncoder->Reset();
        double R = rightEncoder -> GetDistance(); // This won't work, need the gear ratio.
        double L = leftEncoder -> GetDistance();
		double tempangle = 0;

		if(x <= 0) {
			while(tempangle > x) {
			// Gyro needs to have an accepted angle value. +- 5 degrees? or so most likely less, the accepted angle must be based off the distance to the ball we can discuss this today.
				motorControlLeft(-0.5);
				motorControlRight(0.5);
                R = rightEncoder -> GetDistance(); 
                L = leftEncoder -> GetDistance();
                tempangle = getAngleFromTurn(R, L, 0.3); // TEMP 0.3 is the distance from the center of the robot to the wheels.
			}
		} else {
			while( tempangle < x) { // Encoder -> turn radius shit here check with gyro
				motorControlLeft(0.5);
				motorControlRight(-0.5);
                R = rightEncoder -> GetDistance();
                L = leftEncoder -> GetDistance();
                tempangle = getAngleFromTurn(R, L, 0.3);
			}
		}
	}

    void straight(double d) {
        leftEncoder -> Reset();
        rightEncoder -> Reset();
        double x = 0;
        if (d == 0) {

        } else if (d > 0) {
            double templ;
            double tempr;
            while (x < d) {
               motorControlRight(.5);
               motorControlLeft(.5);
               templ = leftEncoder -> GetDistance();
               tempr = rightEncoder -> GetDistance();
               x = (tempr + templ)/2;
            }
            updateCoordinates();
        } else {
            double templ;
            double tempr;
            while (x > d) {
                 motorControlRight(-.5);
                 motorControlLeft(-.5);
                 templ = leftEncoder -> GetDistance();
                 tempr = rightEncoder -> GetDistance();
                 x = (tempr + templ)/2;
            }
            updateCoordinates();
        }
    }

    void updateCoordinates() {
        double R = rightEncoder -> GetDistance();
        double L = leftEncoder -> GetDistance();
        double d = (R+L)/2;
        m_x += d * sin(angle);
        m_y += d * cos(angle);
    }
	
    double getAngleFromTurn(double R, double L, double r) {
        double LAngle = -L/r;
        double RAngle = R/r;
        double NAngle = (LAngle + RAngle)/2;
        return NAngle;
    }

	void DisabledContinuous(void) {

	}
        
	void AutonomousContinuous(void)        
	{

	}

	void TeleopContinuous(void)
	{
	
	}
};

START_ROBOT_CLASS(BuiltinDefaultCode);
