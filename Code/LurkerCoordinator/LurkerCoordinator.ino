#include <JsonParser.h>
#include <JsonGenerator.h>
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
#include <SimpleTimer.h>

using namespace ArduinoJson::Generator;


//////////////////////////////////////////////////////////////////////////
// Unit-Specific Config
const int UNIT_ID = 0;
String unit_class = String("lurker");
String unit_identifer = unit_class + UNIT_ID;


//////////////////////////////////////////////////////////////////////////
// Hardware Config

// Temperature
#define TEMPERATURE_PIN 7
OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature tempSensor(&oneWire);
float temperature;

// Humidity
#define HUMIDITY_PIN 8
#define DHT_TYPE DHT11
DHT humiditySensor(HUMIDITY_PIN, DHT_TYPE);
float humidity;

// Light
BH1750FVI lightSensor;
long illuminance;

// Sound
#define MIC_ANALOG_PIN A0
#define SOUND_OVER_THRESHOLD LOW
#define NOISE_COOLOFF 10	// Cool-off between noise alarms in seconds
int noiseLevel;

// Movement
#define MOTION_PIN 2
#define MOTION_COOLOFF 10 // Cool-off between motion alarms in seconds
#define MOTION_CALIBRATION_TIME 2 //Calibration time in seconds
#define MOVEMENT_DETECTED HIGH
bool motionDetected;


//////////////////////////////////////////////////////////////////////////
// Software Config

SimpleTimer timer;
#define SAMPLE_PERIOD  20000	// Sample period in ms


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
	
	startEnumeration();
}


/**
* Main Loop
*/
void loop(){
	checkSerial();
	checkRadio();
	checkSensors();
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Buffers & Misc

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
// Communications - RF24

/**
* Start the RF24 radio.
* Initiliases listening pipes and allows incoming messages
*/
void initialiseRadio(){
	radio.begin();
	radio.setRetries(15, 5);
	radio.setPALevel(RF24_PA_MAX);
	
	// Start listening on unit-specific pipe
	radio.openReadingPipe(1, unitPipe);
	
	// Listen on broadcast pipe if not the base-station (UNIT 0)
	if (UNIT_ID != 0){
		radio.openReadingPipe(2, BROADCAST_PIPE);
	}
	
	radio.startListening();
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
* Check for incoming packets and act accordingly
*/
void checkRadio(){
	if (radio.available()){
		Serial.print("Message received\n");
		//handleIncomingPacket();
	}
}


/**
* Iterate through the routing table and request a sensor data packets
*/
void requestPackets(){
	// Iterate through the routing table and send packet requests
	for(int i = 0; i < routingTablePutter; i++){
		transmitChar(i, DATA_PACKET_REQUEST);
		
		// Wait for response or timeout
		long transmitTime = millis();
		while (!radio.available() && (millis() - transmitTime) < PACKET_REQUEST_TIMEOUT){
			delay(10);
		}
		
		radio.read(receiveBuffer, RECEIVE_BUFFER_SIZE);
		processReceivedPacket();
	}
}


/**
* Send a network join confirmation to the specified unit
*/
void confirmNetworkJoin(char unitID){
	transmitChar(unitID, NETWORK_JOIN_NOTIFIER);
}


/**
* Process a network join request
* Add the node to the routing table if it doesn't already exist
* Also send a join confirmation to stop the node from spamming
*/
void processNetworkJoin(char unitID){
	if(findRoutingEntry(unitID) == ENTRY_NOT_FOUND){
		addRoutingTableEntry(unitID);
	}
	
	confirmNetworkJoin(unitID);
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
	char packetID = fromReceiveBuffer();
	
	switch(packetID){
		case NETWORK_JOIN_NOTIFIER:
		processNetworkJoin(unitID);
		break;
		
		case DATA_PACKET_RESPONSE:
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


void processDataPacket(int unitID){
	
	Serial.print(SERIAL_PACKET_START);
	Serial.print(unitID);

	while(fromReceiveBuffer() == SERIAL_DIVIDER){
		char dataCode = fromReceiveBuffer();
		float data = decodeData(dataCode);
		
		Serial.print(SERIAL_DIVIDER);
		Serial.print(dataCode);
		Serial.print(data, 2);
	}
}


float decodeData(char dataCode){
	float dataValue;
	
	char high = fromReceiveBuffer();
	char low = fromReceiveBuffer();
	dataValue = word(high, low);

	switch(dataCode){
		case TEMPERATURE_NOTIFICATION:
		case HUMIDITY_NOTIFICATION:
		dataValue = dataValue/100;
		break;
	}
	
}


//////////////////////////////////////////////////////////////////////////
// Communication - Serial

void checkSerial(){
	//TODO Check for serial inputs from the connected Pi/PC
}

void printOpeningMessage(){
	Serial.println("==== Lurker Nano - Coordinator ====");
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

void printSensorData(){
	JsonObject<5> entry;

	entry["id"] = unit_identifer.c_str();
	entry["temperature"] = float(temperature)/100;
	entry["humidity"] = float(humidity)/100;
	entry["illuminance"] = long(illuminance);
	entry["noise"] = long(noiseLevel);

	Serial.print(char(SERIAL_PACKET_START));
	Serial.print(entry);
	Serial.println(char(SERIAL_PACKET_END));
}

void printDataPacket(){
	
	Serial.print(SERIAL_PACKET_START);
	Serial.print(char(UNIT_ID));
	toSendBuffer(SERIAL_DIVIDER);
	
	toSendBuffer(TEMPERATURE_NOTIFICATION);
	toSendBuffer(float(temperature)/100);
	toSendBuffer(SERIAL_DIVIDER);
	
	toSendBuffer(HUMIDITY_NOTIFICATION);
	toSendBuffer(float(humidity)/100);
	toSendBuffer(SERIAL_DIVIDER);
	
	toSendBuffer(LIGHT_NOTIFICATION);
	toSendBuffer(illuminance);
	
	toSendBuffer('\n');
}

/**
* Notify the coordinator that the sound alarm has been tripped
*/
void sendSoundNotification(){
	handleSoundNotification(UNIT_ID);
}


/**
* Alert the coordinator that motion has been detected
*/
void sendMotionNotification(){
	handleMotionNotification(UNIT_ID);
}


/**
* Tell the base station that a sound warning has been detected
*/
void handleSoundNotification(int unitID){
	Serial.print(SERIAL_PACKET_START);
	Serial.print(SOUND_NOTIFICATION);
	Serial.print(SERIAL_DIVIDER);
	Serial.println(unitID);
}


/**
* Tell the base station that a motion warning has been detected
*/
void handleMotionNotification(int unitID){
	JsonObject<2> entry;
	
	entry["id"] = unit_identifer.c_str();
	entry["motion"] = 1;
	
	
	Serial.print(char(SERIAL_PACKET_START));
	Serial.print(entry);
	Serial.println(char(SERIAL_PACKET_END));
}


//////////////////////////////////////////////////////////////////////////
// Sensors

/**
* Start up the Lurker's sensors
*/
void initialiseSensors(){
	tempSensor.begin();
	//humiditySensor.begin();
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
void initialiseLightSensor()
{
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
		//checkHumidity();
		checkLight();
		noiseLevel = getSoundLevel(200);
		
		timeOfSample = millis();
		
		printSensorData();
	}
	
	// Poll presence sensors
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
