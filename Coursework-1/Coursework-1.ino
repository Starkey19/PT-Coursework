#include <LSM303.h>
#include <Vector.h>
#include <NewPing.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <PushButton.h>
#include <QTRSensors.h>

// Ultrasonic sensor pins
#define TRIGGER_PIN 3
#define ECHO_PIN 6
#define MAX_DISTANCE 200 // Max distance to ping for in cm (may need adjusting)

// U/S Sensor
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
unsigned int pingSpeed = 50;	  // Ping frequency  in ms 
unsigned long pingTimer;		  // Keeps track of time since last ping
unsigned int pingCm;			  // Stores ping distance in cm
unsigned int ping_threshold = 3;  // Ping distance threshold for objects, may need adjusting

//Reflectance sensor
#define NUM_SENSORS 6		// Number of reflectance sensors
#define QTR_THRESHOLD 500	// Basic Lighting threshold for black lines
unsigned int sensor_values[NUM_SENSORS];
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

// Motors
#define REVERSE_SPEED     100	// 0 is stopped, 400 is full speed
#define TURN_SPEED        150	// Speed at which to turn
#define FORWARD_SPEED     90	// Speed when going forwards automatically
#define TURN_DURATION     125	// Duration to turn inwards for automatically

#define SCAN_SPEED        100	// Speed at which to turn when scanning

ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // Pin 12 pushbutton
const int motorSpeed = 80;		//Manual foward speed
const int turnSpeed = 150;		//Manual turn speed

int turnCount;							//Counts the automatic turn-ins for boundryChecking errors with corners
unsigned long lastCheckTime;			//Time turn-ins were last checked
unsigned long checkInterval = 2000;		//Interval for checking amount of turn-ins

int incomingByte;      // A variable to read incoming serial data into

//Booleans
bool inRoom;		   // Toggled when entering/in a room
bool stopped;		   // Toggled when motors are stopped
bool boundryCheck;	   // Toggle whether or not to keep inside corridor walls (boundryChecking)
bool returnToStart;	   // Toggle whether to automatically return to the start

//Class to hold room number, the direction the room is on ('L' or 'R') and any objects in room
class Room	
{
public: 
	Room();
	Room(int roomNo, char roomLoc, bool objInRoom);
	char roomLocation;
	bool objectInRoom; 
	int roomNo;
};

//Class to hold information about objects, in particular objects found in corridors
// as they cannot be stored in the room class
class Object 
{
public:
	Object();
	//Nearest prior 'features'
	char afterTurn;
	int afterRoomNo;
};

//Rooms/pathfinding
Vector<Room> rooms;			//STL Vector to store Rooms
Vector<Object> objects;     //STL Vector to store objects

//Stores the last direction turned for finding which side of corridor a room is on 
// and finding the way back
char lastDirection;			

void setup() 
{
	//Start ping timer 
	//pingTimer = millis();

	// Wait for button press before doing anything:
	waitForButton();
	//Calibrates the sensors for the lighting conditions 
	calibrateReflectanceSensors();

	//Set boundryCheck and cornerCheck true intially
	boundryCheck = true; 
	stopped = false;

	returnToStart = false;

	//Set inRoom false initially, lastDirection empty
	inRoom = false;
	lastDirection = ' ';

	turnCount = 0;

	//Wait for button press before doing anything again:
	waitForButton();
}

//Manually calibrate the min/max of the reflectance sensors by rolling them over 
// a black wall/line for 10 seconds
void calibrateReflectanceSensors()
{
	sensors.init();
	delay(500);
	pinMode(13, OUTPUT);
	// Turn on the LED to show we're in calibration mode
	digitalWrite(13, HIGH); 

	unsigned long startTime = millis();
	while (millis() - startTime < 10000) // calibrate sensors for 5 seconds
	{
		sensors.calibrate();
	}
	// turn LED off to show we're through with calibration

	digitalWrite(13, LOW); 

    // Print the calibration minimum values measured when emitters were on
	Serial.begin(9600);
	for (byte i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(sensors.calibratedMinimumOn[i]);
		Serial.print(' ');
	}
	Serial.println();

	// Print the calibration maximum values measured when emitters were on
	for (byte i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(sensors.calibratedMaximumOn[i]);
		Serial.print(' ');
	}
	Serial.println();
	Serial.println();
	delay(1000); // 1 second delay
}

//Waits until the zumo button has been pressed before resuming anything:
void waitForButton()
{
	button.waitForButton();
}

//This was my first attempt at keeping the zumo within the corridor lines adapted from example borderDetect,
// But it resulted in a lot of 'zigzagging' down the corridor and so I decided to use
// the function 'readLine' from example lineFollower instead of raw data to check the position of the zumo relative to lines
void autoBoundryCheckOld(int position) 
{
	sensors.readCalibrated(sensor_values);
	if (sensor_values[2] > 700 || sensor_values[3] > 700) // We hit a corner
	{
		motors.setSpeeds(0, 0);
		Serial.println("Corner!");
		outputSensorData(position);
		boundryCheck = false;
		stopped = true;
	}
	else if (sensor_values[0] > QTR_THRESHOLD)
	{
		motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
		delay(TURN_DURATION);
		motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

	}
	else if (sensor_values[5] > QTR_THRESHOLD)
	{
		motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
		delay(TURN_DURATION);
		motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
	}
}

//Keeps the zumo within the corridor walls by checking the position of the zumo relative to the corridor lines.
// Also looks for corners under sensors 2,3 using raw data - black lines are reliably above 700 reflectance so I
// kept these values static instead of using the CalibratedMaximumOn[2],[3] values.
void autoBoundryCheck(int position)
{
	//Reads the calibrated (from setup) sensor values into my array
	sensors.readCalibrated(sensor_values);

	if (sensor_values[2] > 700 || sensor_values[3] > 700) // Corner
	{
		//Stop the zumo at a corner
		motors.setSpeeds(0, 0);
		Serial.println("Corner!");

		outputSensorData(position); //Todo was for debugging
		
		boundryCheck = false;
		stopped = true;
	}
	else if (position > 0 && position < 2000) // Turn into corridor right
	{
		motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
		delay(TURN_DURATION);
		motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		Serial.print("Turning into corridor right");
		Serial.println();
		outputSensorData(position); //TODO
		//Count amount of turns incase we're stuck in a corner
		turnCount++;
	}
	else if (position > 3500 && position < 5000) // Turn into corridor left
	{
		motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
		delay(TURN_DURATION);
		motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		Serial.print("Turning into corridor left");
		Serial.println();
		outputSensorData(position);	//TODO
		//Count amount of turns incase we're stuck in a corner
		turnCount++;
	} 
	else //Otherwise we should head forwards.
	{
		motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		delay(50);
	}
}

//After entering a room we should do a 360 degree scan using the ultraSonic sensor for objects.
void roomScan() 
{
	stopped = false;
	if (boundryCheck)  
	{
		boundryCheck = false;
	}

	Room newRoom = Room();	//Initialize a new room object

	//Send a message with room number and side of corridor room is on.
	Serial.print("Scanning room no: ");
	Serial.print(newRoom.roomNo, DEC);
	String direction = lastDirection == 'L' ? "Left of corridor" : "Right of corridor";
	Serial.print(" On " + direction);
	Serial.println();
	delay(2);

	unsigned long scanTimer = millis(); // Set scanTimer to sketch duration
	unsigned long scanDuration = 10000; // 10 seconds

	//Scan for 10 seconds whilst turning in a 360 movement to check for any objects in a room
	//If an object is found, send a message indicating this, add it to the current room and stop the zumo.
	while (millis() - scanTimer < scanDuration )
	{
		motors.setLeftSpeed(-SCAN_SPEED);
		motors.setRightSpeed(SCAN_SPEED);

		//pingCm = sonar.ping_cm();	  // Manual check, doesn't need converting to cm

		if ((sonar.ping_cm() < 10) && (sonar.ping_cm() != 0)) // Object found
		{
			pingCm = sonar.ping_cm();
			Serial.print("Object found in room scan! ");
			Serial.println();
			Serial.print(pingCm, DEC);
			Serial.println();
			//Add the object to current room
			newRoom.objectInRoom = true;
			newRoom.roomLocation = lastDirection;
			rooms.push_back(newRoom);
			//Break out of loop and stop zumo
			break;
		}
		delay(10);
	}
	motors.setSpeeds(0, 0);			 //Stop the zumo
}

//A function to check the distance of a ping result against a threshold I defined.
void checkPingDistance(int ping) 
{
	if (ping <= ping_threshold)	//Object found!
	{
		//Stop the zumo
		motors.setSpeeds(0, 0);
		boundryCheck = false;	//Turn off boundryCheck as zumo is stopped
		stopped = true;
		//Create a new object
		Object newObj = Object();
		//Add it to collection
		objects.push_back(newObj);
		//Send message 
		if(!inRoom)
		{
			String direction = newObj.afterTurn == 'L' ? "Left turn" : "Right turn";
			Serial.print("Object found in corridor after " + direction);
			Serial.print(" and after room no ");
			Serial.print(newObj.afterRoomNo, DEC);
			Serial.println();
		}
	}
}

// This function adapted from newPingEventTimer checks every 50ms to see if we pinged the ultrasonic sensor
void echoCheck() 
{
	if (sonar.check_timer() && !stopped) // ping received?
	{
		pingCm = (sonar.ping_result / US_ROUNDTRIP_CM); // Ping returned as uS, conver to cm and store in pingCm
		checkPingDistance(pingCm);						// Function to check the distance from this ping against a set threshold
	}
}

//This method allowed me to RC Control the zumo to collect sensor readings from each sensor to calibrate the code when setting up initially
//and to debug when any errors occurred e.g checking corner detection
void outputSensorData(int position) 
{
	Serial.println();

	Serial.print("5:");
	Serial.print(" ");
	Serial.print(sensor_values[5]);
	Serial.print(" ");

	Serial.print("4:");
	Serial.print(" ");
	Serial.print(sensor_values[4]);
	Serial.print(" ");

	Serial.print("3:");
	Serial.print(" ");
	Serial.print(sensor_values[3]);
	Serial.print(" ");

	Serial.print("2:");
	Serial.print(" ");
	Serial.print(sensor_values[2]);
	Serial.print(" ");

	Serial.print("1:");
	Serial.print(" ");
	Serial.print(sensor_values[1]);
	Serial.print(" ");

	Serial.print("0:");
	Serial.print(" ");
	Serial.print(sensor_values[0]);
	Serial.print(" ");

	Serial.print("Position: ");
	Serial.print(position);
	Serial.println();

	Serial.println();
}

// This method turns the zumo manually left or right when controlling using W,A,S,D
// it then stores the turn direction in the variable lastDirection for 
// detecting which side of the corridor a room is on etc
void turn(char direction)
{
	switch (direction)
	{
	case 'l':	//Left
	case 'L':
		motors.setSpeeds(-turnSpeed, turnSpeed);
		delay(2);
		break;
			
	case 'r':	//Right
	case 'R':
		motors.setSpeeds(turnSpeed, -turnSpeed);
		delay(2);
		break;
	}
	stopped = false;
	lastDirection = direction;
}

void checkForObjects()
{
	if (sonar.ping_cm() > 0)
	{
		if (sonar.ping_cm() <= 10)
		{
			//Stop the zumo
			motors.setSpeeds(0, 0);
			boundryCheck = false;	//Turn off boundryCheck as zumo is stopped
			stopped = true;
			//Create a new object
			Object newObj = Object();
			//Add it to collection
			objects.push_back(newObj);
			//Send message 
			if (!inRoom)
			{
				String direction = newObj.afterTurn == 'L' ? "Left turn" : "Right turn";
				Serial.print("Object found in corridor after " + direction);
				Serial.print(" and after room no ");
				Serial.print(newObj.afterRoomNo, DEC);
				Serial.println();
			}
		}
	}
}

// End of course, optimise return route and navigate to start point.
// Avoid all empty rooms, enter rooms where objects were picked up and send a message if object still there
// LED pin 13 should light up if any objects still present

void courseFinished()
{
	// I wanted to use LSM303.h here and the gyroscope in the zumo to make precision 90 degree turns from the finish line,
	// but without being able to calibrate the gyroscope (possibly due to the xBee shield and weight from ultrasonic sensor and battery)
	// I wasn't able to use the gyroscope.
	returnToStart = true;

	unsigned long turnTimer = millis();
	unsigned long turnDuration = 9000; //Time it takes to turn 180 degrees roughly - but this will rely upon battery charge.

	while (millis() - turnTimer < turnDuration)
	{
		//Turn around
		motors.setLeftSpeed(-100);
		motors.setRightSpeed(100);
		Serial.println("Turning 180");

	}

	boundryCheck = true;

	while (returnToStart)
	{
		// Reads the current position of the zumo relative to the line
		unsigned int position = sensors.readLine(sensor_values);

		autoBoundryCheck(position);

		if (stopped) //We hit a corner
		{
			//TODO lastDirection = corners[cornerCount].reverseDirection;
			//TODO 	cornerCount--;
		}
		//Turn(lastDirection);
		//delay(4500);
		//Motors.setSpeeds(0,0);
	}

}

//Main arduino loop
void loop() 
{	
	//if (millis() >= pingTimer)		 // time(ms) since last ping
	//{
	//	pingTimer += pingSpeed;      // Set the next ping time.
	//	sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
	//}

	if (button.isPressed())
	{
		// Stop if button pressed
		motors.setSpeeds(0, 0);
		button.waitForRelease();
		waitForButton();
	}
	
	// Reads the current position of the zumo relative to the line
	unsigned int position = sensors.readLine(sensor_values); 

	// see if there's incoming serial data:
	if (Serial.available() > 0) 
	{
		// Read the oldest byte in the serial buffer:
		incomingByte = Serial.read();

		//Do a switch on the incoming byte to determine action:
		switch (incomingByte)
		{

		// Toggle task 2 behaviour (automatic boundry checking)
		case 'c': 
			boundryCheck = !boundryCheck; 
			//Stop the zumo motors if we're turning boundryCheck off
			if (!boundryCheck)
			{
				motors.setSpeeds(0, 0);
				stopped = true;
				outputSensorData(position);	//TODO REMOVE
			}
			else
			{
				//If we're boundryChecking we're moving
				inRoom = false;
				stopped = false;
			}
			break;

		//Task 4 - controller stops the zumo to enter a room, 'r' is passed to signify we are entering a room.
		case 'r':
			inRoom = true;
			break;

		// WASD control Task 1 - 
		// right and left turns are handled in Turn() to record the last direction turned

		//Forwards
		case 'w':
			stopped = false;
			motors.setSpeeds(motorSpeed, motorSpeed);
			delay(2);
			break;

		// Left turn
		case 'a':
			turn('L');
			break;

		// Right turn
		case 'd':
			turn('R');
			break;
		//Reverse
		case 's':
			stopped = false;
			motors.setSpeeds(-motorSpeed, -motorSpeed);
			delay(2);
			break;
		//Stop
		case ' ':
			stopped = true;
			motors.setSpeeds(0, 0);
			delay(2);
			break;

		//Scan a room
		case 'x':	
			roomScan();
			break;
		
		//Task 6 - signal that the course has been finished
		case 'e':
			courseFinished();
			break;
		}
	}
	
	//Start autoBoundryChecking and checking for objects TODO
	if (boundryCheck && !inRoom)
	{
		//Check if we're stuck in a corner by counting how many automatic turns are performed within a range of time
		// this seems to be fairly reliable but can't be 100%
		if (millis() - lastCheckTime >= checkInterval)
		{
			lastCheckTime += checkInterval;
			turnCount = 0;
		}
		//If we've turned more than once in this amount of time we're probably in a corner
		if (turnCount > 1)
		{
			//We are almost definitely stuck in a corner!
			motors.setSpeeds(0, 0);
			Serial.println("Stuck in Corner!");

			outputSensorData(position); //TODO

			boundryCheck = false;
			stopped = true;
		}
		checkForObjects();

		if (boundryCheck)
		{
			//Do boundryChecking if we're not in a corner
			autoBoundryCheck(position);
		}
	}
}

//Parameterless constructor for Rooms class
//Initialises the roomNo to size of rooms vector if not empty 
//and other variables to default.
Room::Room()
{
	roomNo = (rooms.size() == 0 ? 1 : rooms.size() + 1);
	roomLocation = ' ';
	objectInRoom = false;

};

Room::Room(int roomNo, char roomLoc, bool objInRoom)
{
	roomNo = roomNo;
	roomLocation = roomLoc;
	objectInRoom = objInRoom;
}
//Parameterless constructor for Objects class
//Initialises afterRoomNo to the last room saved in rooms vector
// and afterTurn to the last manually controlled directional turn
Object::Object()
{
	afterRoomNo = rooms.size();
	afterTurn = lastDirection;
}
