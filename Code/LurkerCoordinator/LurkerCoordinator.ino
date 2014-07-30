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

#define UNIT_ID 0

//////////////////////////////////////////////////////////////////////////
//Communication

// RF24
#define CE_PIN 9
#define CSN_PIN 10
#define LURKER_CHANNEL 90
#define BROADCAST_PIPE 0x90909090FFLL
#define BASE_PIPE 0x9090909000LL
#define MAX_NETWORK_DEVICES 20
#define RF24_TIMEOUT 500

RF24 radio(CE_PIN,CSN_PIN);
const long unitPipe = BASE_PIPE + UNIT_ID;

// Table for storing non-coordinator addresses
char routingTable[MAX_NETWORK_DEVICES];
byte routingTablePutter;
#define ENTRY_NOT_FOUND -1


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
	DATA_PACKET_RESPONSE = 'd'
	
	SOUND_NOTIFICATION = 's',
	MOTION_NOTIFICATION = 'm'
};


// IR
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

// Movement
#define PIR_PIN 7
#define MOVEMENT_DETECTED HIGH

//////////////////////////////////////////////////////////////////////////
// Misc

#define BUZZER_PIN 5

#define LED1_PIN A1
#define LED2_PIN A2
#define LED3_PIN A3

#define BUTTON1_PIN A6
#define BUTTON2_PIN A7

#define SAMPLE_PERIOD 60000

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

//////////////////////////////////////////////////////////////////////////

/**
* Initialization
*/
void setup(){
	Serial.begin(57600);
	Serial.print("Lurker Nano - Coordinator");
	
	initialiseRadio();
	initialiseSensors();
	
	startEnumeration();
}


/**
* Main Loop
*/
void loop()
{
	checkSerial();
	checkRadio();
	checkSensors();
	
}

//////////////////////////////////////////////////////////////////////////


/**
* Start the RF24 radio.
* Initiliases listening pipes and allows incoming messages
*/
void initialiseRadio(){
	radio.begin();
	radio.setChannel(LURKER_CHANNEL);
	radio.setPALevel(RF24_PA_HIGH);
	
	// Start listening on unit-specific pipe
	radio.openReadingPipe(1, unitPipe);
	
	// Listen on broadcast pipe if not the base-station (UNIT 0)
	if (UNIT_ID != 0){
		radio.openReadingPipe(2, BROADCAST_PIPE);
	}
	
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
	
}


/**
* Notify any listening network devices that enumeration has begun.
*/
void startEnumeration(){
	// Flush the routing table
	resetRoutingTable();
	
	// Start the renumeration phase; notify the nodes on the broadcast channel
	radio.stopListening();
	radio.openWritingPipe(BROADCAST_PIPE);
	resetSendBuffer();
	toSendBuffer(NETWORK_ENUMERATION_NOTIFIER);
	for(int i = 0; i < 3; i++){
		radio.write(sendBuffer, sendBufferPutter);
		delay(500);
	}
	
	// Start listening for new join requests
	radio.openReadingPipe(1, BASE_PIPE);
	radio.startListening();
}


void checkSerial(){
	//TODO Check for serial inputs from the connected Pi/PC
}

/**
* Process a network join request
* Add the node to the routing table if it doesn't already exist
* Also send a join confirmation to stop the node from spamming
*/
void processNetworkJoin(char unitID){
	if(findRoutingEntry(unitID) == ENTRY_NOT_FOUND){
		addRoutingTableEntry(unitID);
		confirmNetworkJoin(unitID);
	}
	
	confirmNetworkJoin(unitID);
}


/**
* Send a network join confirmation to the specified unit
*/
void confirmNetworkJoin(char unitID){
	transmitChar(unitID, NETWORK_JOIN_NOTIFIER);
}


/**
* Transmit a character to the specified node
*/
void transmitChar(char unitID, char message){
	radio.stopListening();
	radio.openWritingPipe(BASE_PIPE + unitID);
	resetSendBuffer();
	toSendBuffer(message);
	radio.write(sendBuffer, sendBufferPutter);
	radio.startListening();
}


/**
* Return the address of a unit ID in the routing table
* @return Routing table index of the specified unit ID
* @return -1 if the unit ID is not found
*/
int findRoutingEntry(char unitID){
	int entryIndex = -1;
	
	for(int i = 0; i < MAX_NETWORK_DEVICES; i++){
		if (routingTable[i] == unitID){
			entryIndex = i;
			break;
		}
	}
	
	return entryIndex;
}


/**
* Check for incoming packets and act accordingly
*/
void checkRadio(){
	if (radio.available()){
		handleIncomingPacket();
	}
}


void requestPackets(){
	// Iterate through the routing table and send packet requests
	for(int i = 0; i < routingTablePutter; i++){
		transmitChar(i, DATA_PACKET_REQUEST);
		
		// Wait for response or timeout
		long transmitTime = millis();
		while (!radio.available() && (millis() - transmitTime) < RF24_TIMEOUT){
			delay(10);
		}
		
		radio.read(receiveBuffer, RECEIVE_BUFFER_SIZE);
		processReceivedPacket();
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
	char packetID = fromReceiveBuffer();
	
	switch(packetID){
		case NETWORK_JOIN_NOTIFIER:
		processNetworkJoin(unitID);
		break;
		
		case DATA_PACKET_RESPONSE
		processDataPacket(unitID);
		break;
		
		case SOUND_NOTIFICATION:
		handleSoundNotification(unitID);
		break;
		
		case MOTION_NOTIFICATION:
		handleMotionNotification(unitID);
		break;
	}
}


/**
* Tell the base station that a sound warning has been detected
*/
void handleSoundNotification(int unitID){
	Serial.print("SOUND: Unit ");
	Serial.println(unitID);
}


/**
* Tell the base station that a motion warning has been detected
*/
void handleMotionNotification(int unitID){
	Serial.print("MOTION: Unit ");
	Serial.println(unitID);
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
* Initialise the send buffer
*/
void resetSendBuffer(){
	sendBufferPutter = 0;
}


/**
* Get the next character from the receive buffer
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
* Add a new unit ID to the routing table
* No new entries will be added if the table is already full
*/
void addRoutingTableEntry(char unitID){
	// Only insert character if there is room left in the buffer
	if (routingTablePutter < MAX_NETWORK_DEVICES){
		routingTable[routingTablePutter] = unitID;
		routingTablePutter++;
	}
}


/**
* Flush all routing table entries and ready the table for new entries
*/
void resetRoutingTable(){
	routingTablePutter = 0;
	
	for(int i = 0; i < MAX_NETWORK_DEVICES; i++){
		routingTable[i] = 0;
	}
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
	//TODO Redo sound alart to use serial
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
	//TODO Redo motion notification to use serial
}
