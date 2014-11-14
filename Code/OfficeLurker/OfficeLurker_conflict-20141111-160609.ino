#include <JsonGenerator.h>
#include <Adafruit_Sensor.h>
#include <pgmspace.h>
#include <Adafruit_TSL2561_U.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <DHT.h>
#include "avr/wdt.h"

using namespace ArduinoJson::Generator;

#define UNIT_ID 1

//////////////////////////////////////////////////////////////////////////
//Communication


/**
* Communication Protocol
* Uppercase letters signify messages from the coordinator to the nodes
* Lowercase letters denote messages from nodes to the coordinator
*/
enum COMM_CODES{
	NETWORK_ENUMERATION_NOTIFIER = 'X',
	NETWORK_JOIN_NOTIFIER = 'x',
	NETWORK_JOIN_CONFIRMATION = 'J',
	
	DATA_PACKET_REQUEST = 'D',
	DATA_PACKET_RESPONSE = 'd',
	
	SOUND_NOTIFICATION = 's',
	MOTION_NOTIFICATION = 'm',
	TEMPERATURE_NOTIFICATION = 't',
	HUMIDITY_NOTIFICATION = 'h',
	LIGHT_NOTIFICATION = 'l',
	
	PACKET_START_CHARACTER = '#',
	PACKET_END_CHARACTER = 0x0A,
	SERIAL_DIVIDER = ','
};


//////////////////////////////////////////////////////////////////////////
//Sensors

// Temperature
#define TEMP_PIN 4
#define TEMP_RESOLUTION 12 // Temperature reading resolution in bits
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensors(&oneWire);

// Humidity
#define HUMD_PIN 8
#define DHT_TYPE DHT22
DHT humiditySensor(HUMD_PIN, DHT_TYPE);

// Light
Adafruit_TSL2561_Unified lightSensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);	// Nothing attached to address pin (floating)

// Sound
#define MIC_ANALOG_PIN A0

// Movement
#define MOTION_PIN 7
#define MOTION_COOLOFF 10 // Cool-off between motion alarms in seconds
#define MOTION_CALIBRATION_TIME 10 //Calibration time in seconds
#define MOVEMENT_DETECTED HIGH


//////////////////////////////////////////////////////////////////////////
// Variables
int airTemperature = 0;
int deskTemperature = 0;
int humidity = 0;
int illuminance = 0;
int	noiseLevel = 0;
bool movementDetected = false;

unsigned long timeOfLastMovement = 0;
unsigned long timeOfLastNoise = 0;

long timeOfSample;
#define SAMPLE_PERIOD 6000	// Time between samples in ms

//////////////////////////////////////////////////////////////////////////
// Main Functions
//////////////////////////////////////////////////////////////////////////

/**
* Initialization
*/
void setup(){
	Serial.begin(57600);
	printOpeningMessage();
	
	initialiseSensors();
}


/**
* Main Loop
*/
void loop()
{
	enableWatchdog();
	
	checkSensors();
	wdt_reset();
	delay(500);
	
	disableWatchdog();
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Communication - Serial

void printOpeningMessage(){
	Serial.print("==== Office Lurker - Node #");
	Serial.print(UNIT_ID);
	Serial.print(" ====\n");
}

void printSensorData(){
	JsonObject<6> entry;
	entry["timestamp"] = timeOfSample;
	entry["airTemp"] = float(airTemperature)/100;
	entry["surfaceTemp"] = float(deskTemperature)/100;
	entry["humidity"] = float(humidity)/100;
	entry["illuminance"] = illuminance;
	entry["noiseLevel"] = noiseLevel;

	Serial.print(PACKET_START_CHARACTER);
	Serial.print(entry);
	Serial.print(PACKET_END_CHARACTER);
}

void printMotionEvent(){
	Serial.print("Motion detected - ");
	Serial.println(millis());
}

void printCalibrationMessage(){
	Serial.println("Calibrating motion sensor. Please wait");
}

void printWaitingMessage(){
	Serial.print("...");
}

void printFinishedCalibration(){
	Serial.println("done. Sensor calibrated.");
}


//////////////////////////////////////////////////////////////////////////
// Sensors

/**
* Start up the Lurker's sensors
*/
void initialiseSensors(){
	initialiseTemperatureSensor();
	initialiseHumiditySensor();
	initialiseLightSensor();
	initialiseMotionSensor();
	initialiseSoundSensor();
}


/**
* Initialise the temperature sensor(s)
*/
void initialiseTemperatureSensor(){
	tempSensors.begin();
	tempSensors.setResolution(TEMP_RESOLUTION);
}


/**
* Initialise the humidity sensor
*/
void initialiseHumiditySensor(){
	humiditySensor.begin();
}


/**
* Initialise the BH1750FVI sensor
*/
void initialiseLightSensor(){
	lightSensor.begin();
}


/**
* Calibrate and initialize the PIR motion detector.
* The sensor needs time to take a snapshot reference of the surroundings.
* Stall the program to allow an uninterrupted snapshot.
*/
void initialiseMotionSensor()
{
	pinMode(MOTION_PIN, INPUT);
	printCalibrationMessage();
	
	for(int i = 0; i < MOTION_CALIBRATION_TIME; i++){
		printWaitingMessage();
		delay(1000);
	}
	
	printFinishedCalibration();
	
	delay(50);
}


/**
* Initialise the microphone for sound sensing
*/
void initialiseSoundSensor(){
	pinMode(MIC_ANALOG_PIN, INPUT);
}


/**
* Obtain readings from all of the Lurker's sensors
* Temperature, humidity, and light are sampled periodically
* Sound and motion are polled for presence detection
*
* Results are saved as global variables
*/
void checkSensors(){
	// Periodically check climate sensors
	if ((millis() - timeOfSample) > SAMPLE_PERIOD){
		checkTemperature();
		checkHumidity();
		checkLight();
		noiseLevel = getSoundLevel(200);
		
		timeOfSample = millis();
		
		printSensorData();
	}
	
	// Poll presence sensor
	checkMovement();
}


/**
* Take a temperature reading
* Reading is saved as a shifted decimal integer (12.34 => 1234)
*/
void checkTemperature(){
	float temp;
	tempSensors.requestTemperatures();
	temp = tempSensors.getTempCByIndex(0);
	airTemperature = floatToInt(temp, 2);
	
	temp = tempSensors.getTempCByIndex(1);
	deskTemperature = floatToInt(temp, 2);
}


/**
* Check the relative humidity sensor.
* Output given as a shifted decimal integer (12.34 => 1234)
* The DHT11 takes up to 2 seconds to deliver a response.
*/
void checkHumidity(){
	float hum;
	hum = humiditySensor.readHumidity();
	humidity = floatToInt(hum, 2);
}


/**
* Check the light level hitting the sensor.
* Illuminance saved in lux as an integer
*/
void checkLight(){
	// TSL2561
	sensors_event_t event;
	lightSensor.getEvent(&event);
	illuminance = event.light;
}


/**
* Get the average sound level over the specified sampling period
*
* @param samplePeriod Listening period for the sampling in ms.
* @return Average sound level in 10-bit counts
*/
int getSoundLevel(int samplePeriod){
	unsigned long startTime = millis();
	long total = 0;
	long count = 0;
	
	while (millis() < (startTime + samplePeriod) && samplePeriod > 0){
		int soundLevel = analogRead(MIC_ANALOG_PIN);
		total += soundLevel;
		count += 1;
	}
	
	int average = int(total/count);
	return average;
}


/**
* Check the PIR sensor for detected movement
* The sensor will output HIGH when motion is detected.
* Detections will hold the detection status HIGH until the cool-down has lapsed (default: 60s)
*/
void checkMovement(){
	// Only check the noise alarm if the cool-off has been exceeded
	if((millis() - timeOfLastMovement) > (MOTION_COOLOFF*1000)){
		
		// Check the noise level
		if (digitalRead(MOTION_PIN) == MOVEMENT_DETECTED){
			
			// Noise trigger exceeded, send a notification and start the cool-off
			movementDetected = true;
			timeOfLastMovement = millis();
			printMotionEvent();
			
			}else{
			// No alarm; is fine
			movementDetected = false;
		}
	}
}


//////////////////////////////////////////////////////////////////////////
// Watchdog

/**
* Enable the watchdog timer
* The program will restart if it hangs for more than 8 seconds
*/
void enableWatchdog(){
	// Disable interrupts while setting up the watchdtog
	cli();
	
	// Feed the dog to stop any premature restarts
	wdt_reset();
	
	// Watchdog set to 8 seconds
	wdt_enable(WDTO_8S);

	// Enable interrupts again
	sei();
}


/**
* Disable the watchdog timer
*/
void disableWatchdog(){
	cli();
	
	wdt_reset();
	
	wdt_disable();
	
	sei();
}


/**
* Convert a floating point decimal number into an int
* The decimal is shifted prior to conversion to preserve precision, but reduce range.
* e.g. floatToInt(12.34, 2) = 1234
*
* @return decimal-shifted integer
* @param num Float number to convert
* @param decimalShift Number of decimal shifts (1-4)
*/
int floatToInt(float num, int decimalShift){
	long decimalMultiplier = 1;
	
	if(decimalShift > 0 && decimalShift < 5){
		decimalMultiplier = pow(10, decimalShift);
	}
	
	return (int)(num*decimalMultiplier);
}