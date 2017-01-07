#include <NewPing.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <PushButton.h>
#include <QTRSensors.h>

// Ultrasonic sensor pins
#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 200 // Max distance to ping for in cm (may need adjusting)

// Reflectance sensor
#define NUM_SENSORS 6

ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pin 12 pushbutton

unsigned int sensor_values[NUM_SENSORS];
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

unsigned int pingSpeed = 50; //Ping frequency  in ms 
unsigned long pingTimer;
unsigned int pingCm;
unsigned int ping_threshold = 20; // Ping distance threshold for objects, may need adjusting

const int motorSpeed = 125;
int incomingByte;      // a variable to read incoming serial data into

void setup() 
{
  // initialize serial communication:
  Serial.begin(9600);
  //Start ping timer
  pingTimer = millis();
  // Wait for button press:
  waitForButton();

}

//  TODO: when relectance sensors detect a wall infront of zumo, send a message back over xBee indicating corner reached and disable boundry checking
//	- human will then turn the robot into the room then send a'C' keypress to re-activate automatic boundry checking.

void waitForButton()
{
	button.waitForButton();

	//Start auto boundry movement
	autoBoundryCheck(true);
}

void autoBoundryCheck(bool b) 
{
	if (b) // Activate
	{
		if (sensor_values[0] > 500 && sensor_values[0] < 3500) // Turn inwards left
		{
			motors.setSpeeds(motorSpeed, -motorSpeed);
		}
		else if (sensor_values[5] > 0 && sensor_values[5] < 1500) // Turns inwards right
		{
			motors.setSpeeds(-motorSpeed, motorSpeed);
		}
	}
	else // Deactivate
	{ 
		motors.setSpeeds(0, 0);
	}
}

void checkPingDistance(int ping) // Check ping against a threshold for objects
{
	if (ping < ping_threshold)	//Object found!
	{
		//TODO
	}
}

void echoCheck() // Timer function calls this every 24uS where you can check ping status? TODO
{
	if (sonar.check_timer()) // ping received?
	{
		pingCm = sonar.ping_result / US_ROUNDTRIP_CM;
		checkPingDistance(pingCm);
	}
}

void loop() 
{	
	if (button.isPressed())
	{
		// Stop if button pressed
		motors.setSpeeds(0, 0);
		button.waitForRelease();
		waitForButton();
	}

	// see if there's incoming serial data:
	if (Serial.available() > 0) 
	{
		// read sensor data
		sensors.read(sensor_values);

		// Task 3 - Check if we're at a corner or dead end!
		// TODO - find out which sensors need to be checked for the front ( likely 1-4 ) - will need adjusting 
		if (sensor_values[2] > 400) // Reached a corner
		{
			Serial.println("Reached a corner!");
			autoBoundryCheck(false);
		}

		// read the oldest byte in the serial buffer:
		incomingByte = Serial.read();

		switch (incomingByte)
		{

		//Toggle task 2 behaviour (automatic boundry checking)
		case 'C': 
			autoBoundryCheck(true);
			break;

		//Scan the room using ultrasonic sensor - TODO will this only be run once?
		case 'R':
			if (millis() >= pingTimer) // ms since last ping
			{
				pingTimer += pingSpeed; // set next ping time
				sonar.ping_timer(echoCheck); // send ping
			}
			break;

		case 'W':
			motors.setLeftSpeed(motorSpeed);
			motors.setRightSpeed(motorSpeed);
			delay(2);
			break;

		case 'A':
			motors.setLeftSpeed(-motorSpeed);
			motors.setRightSpeed(motorSpeed);
			delay(2);
			break;

		case 'S':
			motors.setLeftSpeed(-motorSpeed);
			motors.setRightSpeed(-motorSpeed);
			delay(2);
			break;

		case 'D':
			motors.setLeftSpeed(motorSpeed);
			motors.setRightSpeed(-motorSpeed);
			delay(2);
			break;

		case ' ':
			motors.setLeftSpeed(0);
			motors.setRightSpeed(0);
			delay(2);
			break;

		default:
			break;
		}
	}
}