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
#define JOIN_REQUEST_COOLDOWN 2000

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
	MOTION_NOTIFICATION = 'm'
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
#define SOUND_OVER_THRESHOLD HIGH
#define NOISE_COOLOFF 60000	// Cool-off between noise alarms in ms

// Movement
#define MOTION_PIN 7
#define MOTION_COOLOFF 60000	// Cool-off between motion alarms in ms
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

unsigned long timeOfLastMovement;
unsigned long timeOfLastNoise;

#define SEND_BUFFER_SIZE 32
#define RECEIVE_BUFFER_SIZE 32
char sendBuffer[SEND_BUFFER_SIZE];
char receiveBuffer[RECEIVE_BUFFER_SIZE];
byte sendBufferPutter;
byte receiveBufferGetter;

long timeOfSample;
#define SAMPLE_PERIOD 60000


//////////////////////////////////////////////////////////////////////////

/**
* Initialization
*/
void setup(){
	initialiseRadio();
	initialiseSensors();
	joinNetwork();
	enableWatchdog();
}


/**
* Main Loop
*/
void loop()
{
	// Feed the dog
	wdt_reset();
	
	checkRadio();
	
	// Take a sample every minute
	if ((millis() - timeOfSample) > SAMPLE_PERIOD){
		checkSensors();
		timeOfSample = millis();
	}
	else{
		delay(50);
	}
}

//////////////////////////////////////////////////////////////////////////

/**
* Start the RF24 radio.
* Initialise listening pipes and allow incoming messages
*/
void initialiseRadio(){
	radio.begin();
	radio.setChannel(LURKER_CHANNEL);
	radio.setPALevel(RF24_PA_HIGH);
	
	// Open writing channel to coordinator
	radio.openWritingPipe(BASE_PIPE);
	
	// Start listening on unit-specific pipe
	radio.openReadingPipe(1, unitPipe);
	radio.openReadingPipe(2, BROADCAST_PIPE);
	
	radio.startListening();
}


/**
* Start up the Lurker's sensors
*/
void initialiseSensors(){
	tempSensor.begin();
	humiditySensor.begin();
	lightSensor.begin();
	lightSensor.SetMode(Continuous_H_resolution_Mode);
	
	temperature = 0;
	humidity = 0;
	illuminance = 0;
	noiseLevel = 0;
	noiseTriggered = false;
	movementDetected = false;
}


/**
* Check for incoming packets and act accordingly
*/
void checkRadio(){
	if (radio.available()){
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
			break;
			
			case NETWORK_ENUMERATION_NOTIFIER:
			joinNetwork();
			break;
			
			case NETWORK_JOIN_CONFIRMATION:
			isNetworked = true;
			break;
		}
	}
}


/**
* Join the local lurker network - Blocking
* The unit will periodically send join requests until the network has been joined
* Coordinator (UNIT 0) must be running to handle join requests
*/
void joinNetwork(){
	transmitJoinRequest();
	long timeOfLastJoinRequest = millis();
	
	while(!isNetworked){
		if(radio.available()){
			radio.read(receiveBuffer, RECEIVE_BUFFER_SIZE);
			processReceivedPacket();
		}
		
		if (millis() - timeOfLastJoinRequest > JOIN_REQUEST_COOLDOWN){
			transmitJoinRequest();
			
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
	if (isNetworked)
	{
		toSendBuffer(char(UNIT_ID));
		toSendBuffer(char(DATA_PACKET_RESPONSE));
		
		toSendBuffer(temperature);
		toSendBuffer(humidity);
		toSendBuffer(illuminance);
		
		toSendBuffer(char(movementDetected));
		toSendBuffer(char(noiseTriggered));
		
		radio.stopListening();
		radio.write(sendBuffer, sendBufferPutter);
		radio.stopListening();
	}
}


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
* Check if the sound level threshold has been exceeded
* A cool-off period starts each time the sound alarm is tripped.
* The coordinator is notified of each alert.
*/
void checkSound(){
	// Only check the noise alarm if the cool-off has been exceeded
	if((millis() - timeOfLastNoise) > NOISE_COOLOFF){
		
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
* Notify the coordinator that the sound alarm has been tripped
*/
void sendSoundNotification(){
	transmitChar(SOUND_NOTIFICATION);
}


/**
* Check the PIR sensor for detected movement
* The sensor will output HIGH when motion is detected.
* Detections will hold the detection status HIGH until the cool-down has lapsed (default: 60s)
*/
void checkMovement(){
	// Only check the noise alarm if the cool-off has been exceeded
	if((millis() - timeOfLastMovement) > MOTION_COOLOFF){
		
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


/**
* Alert the coordinator that motion has been detected
*/
void sendMotionNotification(){
	transmitChar(MOTION_NOTIFICATION);
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
* Transmit a character to the specified node
*/
void transmitChar(char message){
	if(isNetworked){
		radio.stopListening();
		resetSendBuffer();
		toSendBuffer(message);
		radio.write(sendBuffer, sendBufferPutter);
		radio.startListening();
	}
}
