#include "WPILib.h"
#include <math.h>
#include <sstream>
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
#define robotDiameter 10

// Minimum area of particles to be considered
#define AREA_MINIMUM 500

//Maximum number of particles to process
#define MAX_PARTICLES 8

//Score limits used for hot target determination
#define TAPE_WIDTH_LIMIT 50
#define VERTICAL_SCORE_LIMIT 50
#define LR_SCORE_LIMIT 50

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

// *wind* Flags *wind* (Dicks = true) *Dicks Waving in the Wind*
bool compressor_enabled = true;
bool buttonStartFlag = true;
bool armFlag = true;
bool flag = true;

int potentiometerValue;
int counter = 0;

class BuiltinDefaultCode : public IterativeRobot {


	// Motor controllers
	Talon *left_1;
	Talon *left_2;
	Talon *right_1;
	Talon *right_2;
	Victor *zoidbergRoller;
	Victor *zoidberg_position;
	Victor *shooter;
	
	// Compressor
	Compressor *compressor;	
	
	// Solenoids
	Solenoid *shifter;

	// Gyro
	Gyro *mainGyro;

	// Encoders
	Encoder *leftEncoder;
	Encoder *rightEncoder;

	AnalogChannel *potentiometer;
	AnalogChannel *distanceSensor;

	// Axis Camera
	AxisCamera *camera;

	// Limit switches
	DigitalInput *limitSwitchShooter;

	// Joystick
	Joystick *gamePadDriver; //silver
	Joystick *gamePadShooter; //red

	// Declare (x,y) coordinates of the robot on the field.
	double m_x;
	double m_y;

    // Declare angle of the robot.
    double angle;

    //Structure to represent the scores for the various tests used for target identification
	struct Scores {
		double rectangularity;
		double aspectRatioVertical;
		double aspectRatioHorizontal;
	};

	struct TargetReport {
		int verticalIndex;
		int horizontalIndex;
		bool Hot;
		double totalScore;
		double leftScore;
		double rightScore;
		double tapeWidthScore;
		double verticalScore;
	};

    Scores *scores;
	TargetReport target;
	int verticalTargets[MAX_PARTICLES];
	int horizontalTargets[MAX_PARTICLES];
	int verticalTargetCount, horizontalTargetCount;
	// AxisCamera &camera = AxisCamera::GetInstance();	//To use the Axis camera uncomment this line

public:
	BuiltinDefaultCode(void) {
		printf("BuiltinDefaultCode Constructor Started\n");

		left_1  = new Talon(1);
		left_2  = new Talon(2);
		right_1 = new Talon(3);
		right_2 = new Talon(4);
		// Dicks
		zoidberg_position = new Victor(5);
		shooter = new Victor(7);
		zoidbergRoller = new Victor(8);

		// Compressor
		compressor = new Compressor(3,1);

		shifter = new Solenoid(1, 2);
				
		leftEncoder    = new Encoder(4, 5); // Second int is a placeholder to fix an error with the code (Encoder takes 2 ints)
		rightEncoder   = new Encoder(1, 2); // Same here
		distanceSensor = new AnalogChannel(1);
		potentiometer = new AnalogChannel(2);

		gamePadDriver = new Joystick(1);
		gamePadShooter = new Joystick(2);

		limitSwitchShooter = new DigitalInput(6);

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
	}

	void TeleopInit(void) {
		// int m_telePeriodicLoops = 0; // Reset the loop counter for teleop mode
		// int m_dsPacketsReceivedInCurrentSecond = 0; // Reset the number of dsPackets in current second
		compressor->Start();
		compressor_enabled = true;
	}

	/********************************** Periodic Routines *************************************/

	void DisabledPeriodic(void) {
	}

	void AutonomousPeriodic(void) {
		initialShot(); // 1 is a temporary value.
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
			while(potentiometer->GetValue() >= 0) {
				zoidberg_position->SetSpeed(-speed);
				shooter->Set(0.3);
				left_1 -> Set(0.0);
				left_2 -> Set(0.0);
				right_1 -> Set(0.0);
				right_2 -> Set(0.0);
				zoidbergRoller -> Set(0.75);
			}
			zoidbergRoller->Set(0.0);
		}
		if(position == 1) {
			if(potentiometer->GetValue() < 45) {
				while(potentiometer->GetValue() <= 45) {
					zoidberg_position->SetSpeed(speed);
					shooter->Set(0.0);
					left_1 -> Set(0.0);
					left_2 -> Set(0.0);
					right_1 -> Set(0.0);
					right_2 -> Set(0.0);
					zoidbergRoller -> Set(0.0);
				}
			} else if(potentiometer->GetValue() >= 45) {
				while(potentiometer->GetValue() >= 45) {
					zoidberg_position->SetSpeed(speed);
					shooter->Set(0.0);
					left_1 -> Set(0.0);
					left_2 -> Set(0.0);
					right_1 -> Set(0.0);
					right_2 -> Set(0.0);
					zoidbergRoller -> Set(0.0);
				}
			}
		} else if(position == 2){
			while(potentiometer->GetValue() <= 90){
				zoidberg_position->SetSpeed(speed);
				shooter->Set(0.0);
				left_1 -> Set(0.0);
				left_2 -> Set(0.0);
				right_1 -> Set(0.0);
				right_2 -> Set(0.0);
			}
			zoidbergRoller->SetSpeed(0.75 );
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
		float leftStick  = -1*gamePadDriver->GetRawAxis(2);
		float rightStick = gamePadDriver->GetRawAxis(4);

		// bool buttonX  = gamePadDriver->GetRawButton(1);
		bool buttonA     = gamePadShooter->GetRawButton(2);
		bool buttonB  = gamePadShooter->GetRawButton(3);
		// bool buttonY  = gamePadDriver->GetRawButton(4);

		bool leftBumper  = gamePadDriver->GetRawButton(5);
		bool rightBumper = gamePadDriver->GetRawButton(6);
		
		bool leftTrigger  = gamePadShooter->GetRawButton(7);
		bool rightTrigger = gamePadShooter->GetRawButton(8);

		// bool buttonBack = gamePadDriver->GetRawButton(9);
		bool buttonStart = gamePadDriver->GetRawButton(10);
		float rollerSpeed;
		
		if(buttonStart && buttonStartFlag) {
			if(compressor_enabled) {
				compressor->Stop();
				compressor_enabled = false;
			} else {
				compressor->Start();
				compressor_enabled = true;
			}
			buttonStartFlag = false;
		} else if (!buttonStart) {
			buttonStartFlag = true;
		}
		
		if(rightBumper || leftBumper) {
			if(rightBumper && !leftBumper) {
				ShiftHigh();
			} else if(leftBumper && !rightBumper) {
				ShiftLow();
			}
		
		} else if(rightTrigger && armFlag) {
			armFlag = false;
			moveGobbler(2);
			if(distanceSensor->GetValue() > 90) { // arbitrary value TEST AND FIX
				moveGobbler(0);
			}
		} else if(leftTrigger){
			moveGobbler(0);
		} else if(buttonB){
			rollerSpeed = -1.0;
		} else if(buttonA) {
			shoot();
			flag = false;
		} else if(!buttonA) {
			flag = true;
		} else if(!buttonB){
			armFlag=true;
		}
		

		if(counter % 500) {
			potentiometerValue = potentiometer->GetValue();
			printf("%i potentiometer: %i\n", counter, potentiometerValue);
		}

		counter++;

		// Pickup sequence:
		//
		// User presses a button
		// Zoidberg roller spins up, lowered into a position defined by potentiometer
		// Once reached, it stops lowered
		// If grabber down and button pressed or distance sensor exceeds a certain threshold (moving average) for about 0.5 seconds
		// (Above configurable at the top of the file)
		// Moves back up to previous position while still spinning
		// Once in position, roller stops.

		// My name is Dylan and I like to ingest dicks orally and analy at the same time... For joke go to line 87
		// Line 303 is the best

		// Third position for gobbler between 0 degrees and pickup position when firing
		// Gobbler runs showly when ball is on the catapult

		
		// Motor speed declarations done at the end to ensure watchdog is continually updated.
		motorControlLeft(leftStick);
		motorControlRight(rightStick);
		zoidbergRoller->SetSpeed(rollerSpeed);

		shooter->SetSpeed(0.0);


		// zoidbergRoller->SetSpeed(1.0);
		// zoidbergRoller -> Set(pass(buttonB));
	}
	

	/********************************** Continuous Routines *************************************/

	bool identifyBall(void) {
		
		Threshold threshold(105, 137, 230, 255, 133, 183);	//HSV threshold criteria, ranges are in that order ie. Hue is 60-100
		ParticleFilterCriteria2 criteria[] = {
			{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}
		}; // Particle filter criteria, used to filter out small particles

		while (IsAutonomous() && IsEnabled()) {
            /**
             * Do the image capture with the camera and apply the algorithm described above. This
             * sample will either get images from the camera or from an image file stored in the top
             * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
             */
			ColorImage *image;
			//image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash

			image = camera->GetImage();				//To get the images from the camera comment the line above and uncomment this one
			BinaryImage *thresholdImage = image->ThresholdHSV(threshold);	// get just the green target pixels
			//thresholdImage->Write("/threshold.bmp");
			BinaryImage *filteredImage = thresholdImage->ParticleFilter(criteria, 1);	//Remove small particles
			//filteredImage->Write("Filtered.bmp");

			vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  //get a particle analysis report for each particle

			verticalTargetCount = horizontalTargetCount = 0;
			//Iterate through each particle, scoring it and determining whether it is a target or not
			if(reports->size() > 0)
			{
				scores = new Scores[reports->size()];
				for (unsigned int i = 0; i < MAX_PARTICLES && i < reports->size(); i++) {
					ParticleAnalysisReport *report = &(reports->at(i));

					//Score each particle on rectangularity and aspect ratio
					scores[i].rectangularity = scoreCircularity(report);
					scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, true);
					scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, false);			

					//Check if the particle is a horizontal target, if not, check if it's a vertical target
					if(scoreCompare(scores[i], false))
					{
						printf("particle: %d  is a Horizontal Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
						horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
					} else if (scoreCompare(scores[i], true)) {
						printf("particle: %d  is a Vertical Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
						verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
					} else {
						printf("particle: %d  is not a Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
					}
					printf("Scores rect: %f  ARvert: %f \n", scores[i].rectangularity, scores[i].aspectRatioVertical);
					printf("ARhoriz: %f  \n", scores[i].aspectRatioHorizontal);	
				}

				//Zero out scores and set verticalIndex to first target in case there are no horizontal targets
				target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
				target.verticalIndex = verticalTargets[0];
				for (int i = 0; i < verticalTargetCount; i++)
				{
					ParticleAnalysisReport *verticalReport = &(reports->at(verticalTargets[i]));
					for (int j = 0; j < horizontalTargetCount; j++)
					{
						ParticleAnalysisReport *horizontalReport = &(reports->at(horizontalTargets[j]));
						double horizWidth, horizHeight, vertWidth, leftScore, rightScore, tapeWidthScore, verticalScore, total;

						//Measure equivalent rectangle sides for use in score calculation
						imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &horizWidth);
						imaqMeasureParticle(filteredImage->GetImaqImage(), verticalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &vertWidth);
						imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &horizHeight);

						//Determine if the horizontal target is in the expected location to the left of the vertical target
						leftScore = ratioToScore(1.2*(verticalReport->boundingRect.left - horizontalReport->center_mass_x)/horizWidth);
						//Determine if the horizontal target is in the expected location to the right of the  vertical target
						rightScore = ratioToScore(1.2*(horizontalReport->center_mass_x - verticalReport->boundingRect.left - verticalReport->boundingRect.width)/horizWidth);
						//Determine if the width of the tape on the two targets appears to be the same
						tapeWidthScore = ratioToScore(vertWidth/horizHeight);
						//Determine if the vertical location of the horizontal target appears to be correct
						verticalScore = ratioToScore(1-(verticalReport->boundingRect.top - horizontalReport->center_mass_y)/(4*horizHeight));
						total = leftScore > rightScore ? leftScore:rightScore;
						total += tapeWidthScore + verticalScore;

						//If the target is the best detected so far store the information about it
						if(total > target.totalScore)
						{
							target.horizontalIndex = horizontalTargets[j];
							target.verticalIndex = verticalTargets[i];
							target.totalScore = total;
							target.leftScore = leftScore;
							target.rightScore = rightScore;
							target.tapeWidthScore = tapeWidthScore;
							target.verticalScore = verticalScore;
						}
					}
					//Determine if the best target is a Hot target
					target.Hot = hotOrNot(target);
				}

				if(verticalTargetCount > 0)
				{
					//Information about the target is contained in the "target" structure
					//To get measurement information such as sizes or locations use the
					//horizontal or vertical index to get the particle report as shown below
					ParticleAnalysisReport *distanceReport = &(reports->at(target.verticalIndex));
					double distance = computeDistance(filteredImage, distanceReport);
					if(target.Hot)
					{
						printf("Hot target located \n");
						printf("Distance: %f \n", distance);
					} else {
						printf("No hot target present \n");
						printf("Distance: %f \n", distance);
					}
				}
			}

			// be sure to delete images after using them
			delete filteredImage;
			delete thresholdImage;
			delete image;

			//delete allocated reports and Scores objects also
			delete scores;
			delete reports;
		}
		return false; // T3mporary!!
	}

	bool identifyHotSpots(void)
	{
		Threshold threshold(105, 137, 230, 255, 133, 183);	//HSV threshold criteria, ranges are in that order ie. Hue is 60-100
		ParticleFilterCriteria2 criteria[] = {
			{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}
		}; // Particle filter criteria, used to filter out small particles
		
		while (IsAutonomous() && IsEnabled()) {
            /**
             * Do the image capture with the camera and apply the algorithm described above. This
             * sample will either get images from the camera or from an image file stored in the top
             * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
             */
			ColorImage *image;
			
			// image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash

			image = camera->GetImage();	// To get the images from the camera comment the line above and uncomment this one
			BinaryImage *thresholdImage = image->ThresholdHSV(threshold);	// get just the green target pixels
			//thresholdImage->Write("/threshold.bmp");
			BinaryImage *filteredImage = thresholdImage->ParticleFilter(criteria, 1);	//Remove small particles
			//filteredImage->Write("Filtered.bmp");

			vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  //get a particle analysis report for each particle

			verticalTargetCount = horizontalTargetCount = 0;
			//Iterate through each particle, scoring it and determining whether it is a target or not
			if(reports->size() > 0)
			{
				scores = new Scores[reports->size()];
				for (unsigned int i = 0; i < MAX_PARTICLES && i < reports->size(); i++) {
					ParticleAnalysisReport *report = &(reports->at(i));

					//Score each particle on rectangularity and aspect ratio
					scores[i].rectangularity = scoreRectangularity(report);
					scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, true);
					scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, false);			

					//Check if the particle is a horizontal target, if not, check if it's a vertical target
					if(scoreCompare(scores[i], false))
					{
						printf("particle: %d  is a Horizontal Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
						horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
					} else if (scoreCompare(scores[i], true)) {
						printf("particle: %d  is a Vertical Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
						verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
					} else {
						printf("particle: %d  is not a Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
					}
					printf("Scores rect: %f  ARvert: %f \n", scores[i].rectangularity, scores[i].aspectRatioVertical);
					printf("ARhoriz: %f  \n", scores[i].aspectRatioHorizontal);	
				}

				//Zero out scores and set verticalIndex to first target in case there are no horizontal targets
				target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
				target.verticalIndex = verticalTargets[0];
				for (int i = 0; i < verticalTargetCount; i++)
				{
					ParticleAnalysisReport *verticalReport = &(reports->at(verticalTargets[i]));
					for (int j = 0; j < horizontalTargetCount; j++)
					{
						ParticleAnalysisReport *horizontalReport = &(reports->at(horizontalTargets[j]));
						double horizWidth, horizHeight, vertWidth, leftScore, rightScore, tapeWidthScore, verticalScore, total;

						//Measure equivalent rectangle sides for use in score calculation
						imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &horizWidth);
						imaqMeasureParticle(filteredImage->GetImaqImage(), verticalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &vertWidth);
						imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &horizHeight);

						//Determine if the horizontal target is in the expected location to the left of the vertical target
						leftScore = ratioToScore(1.2*(verticalReport->boundingRect.left - horizontalReport->center_mass_x)/horizWidth);
						//Determine if the horizontal target is in the expected location to the right of the  vertical target
						rightScore = ratioToScore(1.2*(horizontalReport->center_mass_x - verticalReport->boundingRect.left - verticalReport->boundingRect.width)/horizWidth);
						//Determine if the width of the tape on the two targets appears to be the same
						tapeWidthScore = ratioToScore(vertWidth/horizHeight);
						//Determine if the vertical location of the horizontal target appears to be correct
						verticalScore = ratioToScore(1-(verticalReport->boundingRect.top - horizontalReport->center_mass_y)/(4*horizHeight));
						total = leftScore > rightScore ? leftScore:rightScore;
						total += tapeWidthScore + verticalScore;

						//If the target is the best detected so far store the information about it
						if(total > target.totalScore)
						{
							target.horizontalIndex = horizontalTargets[j];
							target.verticalIndex = verticalTargets[i];
							target.totalScore = total;
							target.leftScore = leftScore;
							target.rightScore = rightScore;
							target.tapeWidthScore = tapeWidthScore;
							target.verticalScore = verticalScore;
						}
					}
					//Determine if the best target is a Hot target
					target.Hot = hotOrNot(target);
				}

				if(verticalTargetCount > 0)
				{
					//Information about the target is contained in the "target" structure
					//To get measurement information such as sizes or locations use the
					//horizontal or vertical index to get the particle report as shown below
					ParticleAnalysisReport *distanceReport = &(reports->at(target.verticalIndex));
					double distance = computeDistance(filteredImage, distanceReport);
					if(target.Hot)
					{
						printf("Hot target located \n");
						printf("Distance: %f \n", distance);
					} else {
						printf("No hot target present \n");
						printf("Distance: %f \n", distance);
					}
				}
			}

			// be sure to delete images after using them
			delete filteredImage;
			delete thresholdImage;
			delete image;

			//delete allocated reports and Scores objects also
			delete scores;
			delete reports;
		}
		return false; // T3mporary!!
	}
	
	double computeDistance (BinaryImage *image, ParticleAnalysisReport *report) {
		double rectShort, height;
		int targetHeight;
	
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
		//using the smaller of the estimated rectangle short side and the bounding rectangle height results in better performance
		//on skewed rectangles
		height = min(report->boundingRect.height, rectShort);
		targetHeight = 32;
	
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

	bool scoreCompare(Scores scores, bool vertical){
		bool isTarget = true;

		isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
		if(vertical){
			isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
		} else {
			isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
		}

		return isTarget;
	}

	double scoreRectangularity(ParticleAnalysisReport *report){
		if(report->boundingRect.width*report->boundingRect.height !=0){
			return 100*report->particleArea/(report->boundingRect.width*report->boundingRect.height);
		} else {
			return 0;
		}	
	}

	double scoreCircularity(ParticleAnalysisReport *report)
	{
		if(report->boundingRect.width*report->boundingRect.height !=0){
			return 100*(report->boundingRect.width / 2) * (report->boundingRect.width / 2) * PI / ((report->boundingRect.width / 2) * (report->boundingRect.width / 2) * PI);
		} else {
			return 0;
		}
	}
	
	double ratioToScore(double ratio)
	{
		return (max(0, min(100*(1-fabs(1-ratio)), 100)));
	}

	bool hotOrNot(TargetReport target)
	{
		bool isHot = true;

		isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
		isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
		isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);

		return isHot;
	}

	void initialShot() {
		if(hotOrNot){
		straight(500); // temp value
		shoot();
	}else{
		straight(-500); //temp value
		turn(-PI/2); // turn 90
		straight(1000); // drive to other side
		turn(PI/2); // turn 90
		// maybe drive forward using distance sensor?
		// meh should be fine
		
		shoot();
	}
		
		
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
		int stage=0;
		zoidbergAnalog->GetValue();
		
		while(limitSwitch->Get()){ // not pressed
			int shooterSpeed=1.0;
			int positionSpeed =0.0;
			if(zoidbergAnalog<90){
				positionSpeed->SetSpeed(0.5); //90 is a temp value
			}
			Shooter->SetSpeed(shooterSpeed);
			motorControlLeft(0);
			motorControlRight(0);
			zoidbergRoller->SetSpeed(0);
			zoidbergPosition->SetSpeed(positionSpeed);
		}
		shooter->SetSpeed(0.0);
			
	
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
                tempangle = getAngleFromTurn(R, L, robotDiameter); // TEMP 0.3 is the distance from the center of the robot to the wheels.
			}
		} else {
			while( tempangle < x) { // Encoder -> turn radius shit here check with gyro
				motorControlLeft(0.5);
				motorControlRight(-0.5);
                R = rightEncoder -> GetDistance();
                L = leftEncoder -> GetDistance();
                tempangle = getAngleFromTurn(R, L, robotDiameter);
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
