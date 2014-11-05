#include <IRremote.h>
#include <SPI.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <RF24_config.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <OneWire.h>
#include <BH1750FVI.h>
#include <DHT.h>
#include "avr/wdt.h"

// Unit ID (1 - 20)
#define UNIT_ID 1

//////////////////////////////////////////////////////////////////////////
//Communication

// RF24
#define CE_PIN 9
#define CSN_PIN 10
#define LURKER_CHANNEL 90
#define BROADCAST_PIPE 0x90909090FFLL
#define BASE_PIPE 0x9090909000LL
#define NETWORK_COORDINATOR 0
#define RF24_TIMEOUT 500
#define JOIN_TIMEOUT 1000
#define JOIN_REQUEST_COOLDOWN 10 // Minimum time between join requests in seconds
#define NETWORK_TIMEOUT 600	// Network inactivity timeout in seconds

RF24 radio(CE_PIN,CSN_PIN);
const long unitPipe = BASE_PIPE + UNIT_ID;
bool isNetworked = false;

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
	
	SERIAL_PACKET_START = '#',
	SERIAL_DIVIDER = ','
};


// IR
//TODO Implement the IR blaster and receiver
#define IR_TX_PIN 3
#define IR_RX_PIN 2
//IRsend irSend(IR_TX_PIN);


//////////////////////////////////////////////////////////////////////////
//Sensors

// Temperature
#define TEMP_PIN 4
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);

// Humidity
#define HUMD_PIN 8
#define DHT_TYPE DHT11
DHT humiditySensor(HUMD_PIN, DHT_TYPE);

// Light
BH1750FVI lightSensor;

// Sound
#define MIC_DIGITAL_PIN 6
#define MIC_ANALOG_PIN A0
#define SOUND_OVER_THRESHOLD LOW
#define NOISE_COOLOFF 10	// Cool-off between noise alarms in seconds

// Movement
#define MOTION_PIN 7
#define MOTION_COOLOFF 10 // Cool-off between motion alarms in seconds
#define MOTION_CALIBRATION_TIME 10 //Calibration time in seconds
#define MOVEMENT_DETECTED HIGH

//////////////////////////////////////////////////////////////////////////
// Misc

#define BUZZER_PIN 5

#define LED1_PIN A1
#define LED2_PIN A2
#define LED3_PIN A3

#define BUTTON1_PIN A6
#define BUTTON2_PIN A7

//////////////////////////////////////////////////////////////////////////
// Variables
int temperature;
int humidity;
int illuminance;
int	noiseLevel;
bool noiseTriggered;
bool movementDetected;

unsigned long timeOfLastMovement = 0;
unsigned long timeOfLastNoise = 0;

unsigned long timeOfLastPacket = 0;
unsigned long timeOfLastJoinRequest = 0;

#define SEND_BUFFER_SIZE 32
#define RECEIVE_BUFFER_SIZE 32
char sendBuffer[SEND_BUFFER_SIZE];
char receiveBuffer[RECEIVE_BUFFER_SIZE];
byte sendBufferPutter;
byte receiveBufferGetter;

long timeOfSample;
#define SAMPLE_PERIOD 6000

//////////////////////////////////////////////////////////////////////////
// Main Functions
//////////////////////////////////////////////////////////////////////////

/**
* Initialization
*/
void setup(){
	Serial.begin(57600);
	printOpeningMessage();
	
	initialiseRadio();
	initialiseSensors();
}


/**
* Main Loop
*/
void loop()
{
	enableWatchdog();
	
	checkNetwork();
	wdt_reset();
	
	checkRadio();
	checkSensors();
	wdt_reset();
	
	delay(500);
	disableWatchdog();
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Buffer & Misc

/**
* Insert a character into the send buffer
*/
void toSendBuffer(char c){
	// Only insert character if there is room left in the buffer
	if (sendBufferPutter < SEND_BUFFER_SIZE){
		sendBuffer[sendBufferPutter] = c;
		sendBufferPutter++;
	}
}


/**
* Insert an integer (2 bytes) into the send buffer
*/
void toSendBuffer(int i){
	toSendBuffer(char(highByte(i)));
	toSendBuffer(char(lowByte(i)));
}


/**
* Initialise the send buffer
*/
void resetSendBuffer(){
	sendBufferPutter = 0;
}


/**
* Get the next character from the receive buffer
* Will return the last character of the buffer if the end has already been reached
*/
char fromReceiveBuffer(){
	char c = receiveBuffer[receiveBufferGetter];
	
	if (receiveBufferGetter < RECEIVE_BUFFER_SIZE){
		receiveBufferGetter++;
	}
	
	return c;
}


/**
* Initialize the receive buffer by resetting the getter position
*/
void resetReceiveBuffer(){
	receiveBufferGetter = 0;
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


//////////////////////////////////////////////////////////////////////////
// Communication - RF24 Radio

/**
* Start the RF24 radio.
* Initialise listening pipes and allow incoming messages
*/
void initialiseRadio(){
	radio.begin();
	
	radio.setChannel(LURKER_CHANNEL);
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_2MBPS);
	radio.setRetries(2, 15);
	
	// Open writing channel to coordinator
	radio.openWritingPipe(BASE_PIPE);
	
	// Start listening on unit-specific pipe
	radio.openReadingPipe(1, unitPipe);
	radio.openReadingPipe(2, BROADCAST_PIPE);
	
	radio.startListening();
}


/**
* Check for incoming packets and act accordingly
*/
void checkRadio(){
	if (radio.available()){
		timeOfLastPacket = millis();
		handleIncomingPacket();
	}
}


/**
* Read and process incoming packets from the RF24 radio
*/
void handleIncomingPacket(){
	resetReceiveBuffer();
	resetSendBuffer();
	radio.read(receiveBuffer, RECEIVE_BUFFER_SIZE);
	processReceivedPacket();
}


/**
* Respond to the latest incoming packet
* The packet must already be read into the received buffer
*/
void processReceivedPacket(){
	char unitID = fromReceiveBuffer();
	
	// Only accept commands from the coordinator (UNIT 0)
	if(unitID == NETWORK_COORDINATOR){
		char packetID = fromReceiveBuffer();
		
		switch(packetID){
			case DATA_PACKET_REQUEST:
			transmitDataPacket();
			printSentPacket();
			break;
			
			case NETWORK_ENUMERATION_NOTIFIER:
			checkNetwork();
			break;
			
			case NETWORK_JOIN_CONFIRMATION:
			isNetworked = true;
			printJoinNotification();
			break;
		}
	}
}


/**
* Join the local lurker network - Blocking
* The unit will periodically send join requests until the network has been joined
* Coordinator (UNIT 0) must be running to handle join requests
*/
void checkNetwork(){
	
	// Remove network status if a packet hasn't been received in a while
	if(isNetworked && (millis() - timeOfLastPacket) > (NETWORK_TIMEOUT*1000)){
		printNetworkTimeout();
		isNetworked = false;
	}
	
	// Check if the network has been joined
	if(!isNetworked && (millis() - timeOfLastJoinRequest) > (JOIN_REQUEST_COOLDOWN*1000)){
		attemptNetworkJoin();
	}
}


/**
* Request to join the network and wait for the response
* Timeout is 500ms
*/
void attemptNetworkJoin(){
	// Attempt to join the network
	transmitJoinRequest();
	timeOfLastJoinRequest = millis();
	printJoinRequest();
	
	// Check for response
	while(!isNetworked && (millis() - timeOfLastJoinRequest) < JOIN_TIMEOUT){
		if(radio.available()){
			radio.read(receiveBuffer, RECEIVE_BUFFER_SIZE);
			processReceivedPacket();
		}
	}
}


/**
* Transmit a join request to the lurker coordinator
*/
void transmitJoinRequest(){
	transmitChar(NETWORK_JOIN_NOTIFIER);
}


/**
* Transmit the latest sensor data to the coordinator
* Data packet structure - [Unit ID] [Packet ID] [Temp][] [Humidity][] [Light][] [Sound][Movement]
*/
void transmitDataPacket(){
	if (isNetworked){
		resetSendBuffer();
		
		toSendBuffer(char(UNIT_ID));
		toSendBuffer(char(DATA_PACKET_RESPONSE));
		toSendBuffer(SERIAL_DIVIDER);
		
		toSendBuffer(TEMPERATURE_NOTIFICATION);
		toSendBuffer(temperature);
		toSendBuffer(SERIAL_DIVIDER);
		
		toSendBuffer(HUMIDITY_NOTIFICATION);
		toSendBuffer(humidity);
		toSendBuffer(SERIAL_DIVIDER);
		
		toSendBuffer(LIGHT_NOTIFICATION);
		toSendBuffer(illuminance);
		
		toSendBuffer('\n');
		
		radio.stopListening();
		radio.write(sendBuffer, sendBufferPutter);
		radio.stopListening();
	}
}


/**
* Transmit a character to the specified node
*/
void transmitChar(char message){
	radio.stopListening();
	resetSendBuffer();
	toSendBuffer(message);
	radio.write(sendBuffer, sendBufferPutter);
	radio.startListening();
}


/**
* Notify the coordinator that the sound alarm has been tripped
*/
void sendSoundNotification(){
	printSoundEvent();
	transmitChar(SOUND_NOTIFICATION);
}


/**
* Alert the coordinator that motion has been detected
*/
void sendMotionNotification(){
	printMotionEvent();
	transmitChar(MOTION_NOTIFICATION);
}


//////////////////////////////////////////////////////////////////////////
// Communication - Serial

void printOpeningMessage(){
	Serial.print("==== Lurker Nano - Node #");
	Serial.print(UNIT_ID);
	Serial.print(" ====\n");
}

void printSensorData(){
	Serial.print("\n===== Sensor Data =====");
	Serial.print("\nTemperature:\t");
	Serial.print(float(temperature)/100);
	Serial.print(" C\nHumidity:\t");
	Serial.print(float(humidity)/100);
	Serial.print(" %\nLight:\t\t");
	Serial.print(illuminance);
	Serial.print(" lux\nSound Level:\t");
	Serial.print(noiseLevel);
	Serial.print(" count\n\n");
}

void printMotionEvent(){
	Serial.println("Motion detected");
}

void printSoundEvent(){
	Serial.println("Sound detected");
}

void printNetworkTimeout(){
	Serial.println("Network timeout");
}

void printJoinRequest(){
	Serial.println("Network join request sent...");
}

void printJoinNotification(){
	Serial.println("Joined network");
}

void printSentPacket(){
	Serial.println("Send data packet to coordinator");
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
	tempSensor.begin();
	humiditySensor.begin();
	initialiseLightSensor();
	initialiseMotion();
	initialiseSound();
	
	
	// Initialise variables
	temperature = 0;
	humidity = 0;
	illuminance = 0;
	noiseLevel = 0;
	noiseTriggered = false;
	movementDetected = false;
}


/**
* Initialise the BH1750FVI sensor
*/
void initialiseLightSensor(){
	lightSensor.begin();
	lightSensor.SetAddress(Device_Address_L);
	lightSensor.SetMode(Continuous_H_resolution_Mode);
	
}


/**
* Calibrate and initialize the PIR motion detector.
*/
void initialiseMotion()
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
void initialiseSound(){
	pinMode(MIC_DIGITAL_PIN, INPUT);
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
	
	// Poll presence sensors
	checkSound();
	checkMovement();
}


/**
* Take a temperature reading
* Reading is saved as a shifted decimal integer (12.34 => 1234)
*/
void checkTemperature(){
	float temp;
	tempSensor.requestTemperatures();
	temp = tempSensor.getTempCByIndex(0);
	temperature = floatToInt(temp, 2);
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
	illuminance = lightSensor.GetLightIntensity();
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
* Check if the sound level threshold has been exceeded
* A cool-off period starts each time the sound alarm is tripped.
* The coordinator is notified of each alert.
*/
void checkSound(){
	// Only check the noise alarm if the cool-off has been exceeded
	if((millis() - timeOfLastNoise) > (NOISE_COOLOFF*1000)){
		
		// Check the noise level
		if (digitalRead(MIC_DIGITAL_PIN) == SOUND_OVER_THRESHOLD){
			
			// Noise trigger exceeded, send a notification and start the cool-off
			noiseTriggered = true;
			timeOfLastNoise = millis();
			sendSoundNotification();
			
			}else{
			// No alarm; is fine
			noiseTriggered = false;
		}
	}
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
			sendMotionNotification();
			
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


