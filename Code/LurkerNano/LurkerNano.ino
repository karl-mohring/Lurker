#include <dht.h>
#include <CommandHandler.h>
#include <RF24_config.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Logging.h>
#include <SPI.h>
#include <StraightBuffer.h>
#include <SimpleTimer.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BH1750FVI.h>
#include <JsonGenerator.h>
#include "avr/wdt.h"
#include "avr/pgmspace.h"
#include "lurker_settings.h"

using namespace ArduinoJson::Generator;

const float LURKER_VERSION = 0.9;

//////////////////////////////////////////////////////////////////////////
// Lurker Nano
//
// The lurker is a small and inexpensive WSN node for home climate
// monitoring.
//
// Sensors:
//	- Temperature
//	- Humidity
//	- Illuminance
//	- Motion
//	- IR Receiver (not implemented)
//
// Actuators:
//	- Buzzer (active)
//	- IR Blaster
//	- 2 LEDs
//
//////////////////////////////////////////////////////////////////////////
// Hardware Config

// Temperature
OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature tempSensor(&oneWire);

// Humidity
dht humiditySensor;

// Illuminance
BH1750FVI lightSensor;

// Motion
bool motionDetected;
long timeOfLastMotion;

// Buzzer
RF24 radio(CE_PIN, CSN_PIN);

// Lights
int leds[] = { LED0, LED1, LED_BUILTIN };


//////////////////////////////////////////////////////////////////////////
// Soft Config

// Sample Timer
SimpleTimer timer;
int joinTimerID;
int networkTimeoutTimerID;
int printDataTimerID;
int motionTimerId;

// Communication
bool connectedToNetwork = false;

String received = "";
bool recording = false;

// Radio Buffer
byte _readBuffer[BUFFER_LENGTH];
byte _writeBuffer[BUFFER_LENGTH];
StraightBuffer readBuffer(_readBuffer, BUFFER_LENGTH);
StraightBuffer writeBuffer(_writeBuffer, BUFFER_LENGTH);

// Coordinator - Routing table
int routingTable[MAX_NETWORK_SIZE];

// Sensor data object
JsonObject<8> sensorData;
JsonObject<5> remoteData;

// Command handlers
char _commandCache[BUFFER_LENGTH];
CommandHandler commandHandler(_commandCache, BUFFER_LENGTH);

// Logger 
char p_buffer[80];
#define P(str) (strcpy_P(p_buffer, PSTR(str)), p_buffer)


//////////////////////////////////////////////////////////////////////////
// Main Functions
//////////////////////////////////////////////////////////////////////////

/**
* Initial script - Run once
*/
void setup()
{
	// Starting up the Lurker - say hello :D
	Log.Init(LOGGER_LEVEL, SERIAL_BAUD);
	Log.Info(P("Lurker starting - %s"), unitID.c_str());

	// Start all the things
	startSensors();
	initialiseBuzzer();
	initialiseRadio();
	initialiseLights();
	startCommandHandler();
}

/**
* Main Loop
*/
void loop(){
	timer.run();

	checkSerial();
	//checkRadio();
}


//////////////////////////////////////////////////////////////////////////
// Communication - Wired

// Periodic function - Transmit sensor data over serial
/**
* Send sensor data to the connected device
* Sensor data is structured in JSON format
*/
void printSensorData(){
	readSensors();

	Serial.print(PACKET_START);
	Serial.print(sensorData);
	Serial.println(PACKET_END);
}

/**
* Check serial for incoming packets
*/
void checkSerial(){
	while (Serial.available()){
		char inChar = Serial.read();
		commandHandler.readIn(inChar);
	}
}

/**
* Bind commands for the serial and rf24 interface
*/
void startCommandHandler(){
	Log.Debug(P("Adding commands"));

	commandHandler.setTerminator(PACKET_END);
	Log.Info(P("Terminate characters with a '%c' character"), PACKET_END);
	commandHandler.setDefaultHandler(commandNotRecognised);

	// User functions
	commandHandler.addCommand(BUZZER_ON_CODE, buzzerOn);
	Log.Info(P("%c - Buzzer ON"), BUZZER_ON_CODE);

	commandHandler.addCommand(BUZZER_OFF_CODE, buzzerOff);
	Log.Info(P("%c - Buzzer OFF"), BUZZER_OFF_CODE);

	commandHandler.addCommand(SENSOR_READ_REQUEST, printSensorData);
	Log.Info(P("%c - Read sensors"), SENSOR_READ_REQUEST);

	// Internal functions
	if (UNIT_NUMBER == COORDINATOR){
		commandHandler.addCommand(NETWORK_JOIN_REQUEST, addNodeToNetwork);
		commandHandler.addCommand(DATA_TRANSMIT_RESPONSE, resetRemoteDataStorage);
		commandHandler.addCommand(UNIT_ID_CODE, readRemoteUnitNumber);
		commandHandler.addCommand(TEMPERATURE_CODE, readRemoteTemperature);
		commandHandler.addCommand(HUMIDITY_CODE, readRemoteHumidity);
		commandHandler.addCommand(ILLUMINANCE_CODE, readRemoteIlluminance);
		commandHandler.addCommand(MOTION_CODE, readRemoteMotion);
		commandHandler.addCommand(DATA_PACKET_FINISHED, processRemoteDataPacket);
	}

	else{
		commandHandler.addCommand(NETWORK_JOIN_CONFIRM, processNetworkJoin);
		commandHandler.addCommand(DATA_TRANSMIT_REQUEST, transmitDataPacket);
	}
}

/**
* Callback for an unknown command
*/
void commandNotRecognised(const char command){
	Log.Error(P("Warning - Unknown command [%c]"), char(command));
}


//////////////////////////////////////////////////////////////////////////
// Communication - Wireless

/**
* Initialise the RF24 radio
*/
void initialiseRadio(){
	// Set up the options for the transceiver
	radio.begin();
	radio.setDataRate(RF24_1MBPS);
	radio.setPALevel(RF24_PA_MAX);
	radio.setRetries(15, 15);
	connectedToNetwork = false;

	// Open communication channels
	radio.openWritingPipe(BASE_PIPE);

	radio.openReadingPipe(1, UNIT_PIPE);
	Log.Debug(P("Reading pipe: %l"), UNIT_PIPE);

	radio.openReadingPipe(2, BROADCAST_PIPE);
	Log.Debug(P("Reading pipe: %l"), BROADCAST_PIPE);

	// Set up network joining
	if (UNIT_NUMBER != COORDINATOR){
		timer.setInterval(NETWORK_JOIN_INTERVAL, joinNetwork);
	}

	radio.startListening();

	Log.Debug(P("Radio started"));
}

/**
* Check the radio for any incoming packets.
*/
void checkRadio(){
	if (radio.available()){

		Log.Info(P("Packet received"));
		readRadioPacket();

		// Get rid of any unfinished serial commands in the buffer
		commandHandler.clearCache();

		// Read in the packet, one byte at a time
		char command = readBuffer.read();
		while (readBuffer.available() && command != 0){
			command = readBuffer.read();
			commandHandler.readIn(command);

			Log.Debug(P("Command: [%i]"), command);
		}
	}
}

/**
* Read the incoming radio packet into the read buffer
*/
void readRadioPacket(){
	radio.read(readBuffer.getBufferAddress(), BUFFER_LENGTH);
	readBuffer.setWritePosition(BUFFER_LENGTH);
}

/**
* Transmit a character to the specified node
*/
void transmitChar(char message){
	radio.stopListening();
	radio.write(&message, 1);
	radio.startListening();

	Log.Verbose(P("Sending: %c"), message);
}


//////////////////////////////////////////////////////////////////////////
// Coordinator Functions

/**
* Change a units' status to active in the routing table
*/
void addNodeToNetwork(){
	char unitNum = commandHandler.next();

	// Units are numbered sequentially, make sure the number is not too high
	if (unitNum <= MAX_NETWORK_SIZE){
		routingTable[unitNum] = timer.setTimeout(NODE_TIMEOUT, cleanRoutingTable);
		Log.Info(P("Unit %i joined the network"), unitNum);
	}
}

/**
* Go through the routing table and disable units that have timed out
*/
void cleanRoutingTable(){
	for (int i = 0; i < MAX_NETWORK_SIZE; i++){

		// Active units have a timer ID higher than -1.
		if (routingTable[i] >= 0){

			// Clear any units from the routing table that are non-responsive
			if (!timer.isEnabled(routingTable[i])){
				Log.Info(P("Unit %i timed out"), i);
				routingTable[i] = -1;
			}
		}
	}
}

/**
* Reset the remote data values to prepare for a new packet
*/
void resetRemoteDataStorage(){
	remoteData[ID] = "None";
	remoteData[TEMPERATURE] = 0.0;
	remoteData[HUMIDITY] = 0;
	remoteData[ILLUMINANCE] = 0;
	remoteData[MOTION] = false;
}

/**
* Read in the unit number from the received packet
*/
void readRemoteUnitNumber(){
	char arg = commandHandler.next();
	int unitNumber = byte(arg);

	if (unitNumber < MAX_NETWORK_SIZE){
		remoteData[ID] = unitNumber;

		// Reset the timeout of the sender
		resetTimeout(unitNumber);
	}
}

/**
* Read in the temperature from the received packet
*/
void readRemoteTemperature(){
	char argHigh = commandHandler.next();
	char argLow = commandHandler.next();
	float temperature = word(argHigh, argLow);
	temperature /= 100.0;

	remoteData[TEMPERATURE] = temperature;
}

/**
* Read in the humidity from the received packet
*/
void readRemoteHumidity(){
	char argHigh = commandHandler.next();
	char argLow = commandHandler.next();
	float humidity = word(argHigh, argLow);
	humidity /= 100.0;

	remoteData[HUMIDITY] = humidity;
}

/**
* Read in the illuminance from the received packet
*/
void readRemoteIlluminance(){
	char argHigh = commandHandler.next();
	char argLow = commandHandler.next();
	int illuminance = word(argHigh, argLow);

	remoteData[ILLUMINANCE] = illuminance;
}

/**
* Read in the motion detector status from the received packet
*/
void readRemoteMotion(){
	bool motion = commandHandler.next();

	remoteData[MOTION] = motion;
}

/**
* Print the received data values to the buffer
*/
void processRemoteDataPacket(){
	Serial.print(PACKET_START);
	Serial.print(remoteData);
	Serial.println(PACKET_END);
}

/**
* Reset the timeout of a node
*/
void resetTimeout(int unitNumber){
	timer.restartTimer(routingTable[unitNumber]);
}

/**
* Callback function
* Request a data packet from a remote node.
*
* Requests are made sequentially through the routing table.
* If a node is not active, nothing happens, rather than skipping ahead to an active node.
* It's a lot less spammy that way...
*
* Also, the coordinator will be in its own routing table, but the coordinator has no methods
* to join its own network. It should be fine...
* The break in requests gives the coordinator a chance to upload its own data.
*/
void requestNodeData(){
	static int index = 0;

	// Get data from the next connected node
	if (routingTable[index] >= 0){
		Log.Info(P("Requesting data from Unit %i"), index);
		sendPacketRequest(index);
	}

	// Increment node number or loop back to start
	if (index < MAX_NETWORK_SIZE){
		index++;
	}
	else{
		index = 0;
	}
}

/**
* Send a data packet request to the specified node
* @param unitNumber Unit number of the destination node
*/
void sendPacketRequest(int unitNumber){
	writeBuffer.reset();

	writeBuffer.write(DATA_TRANSMIT_REQUEST);
	writeBuffer.write(PACKET_END);

	radio.openWritingPipe(BASE_PIPE + unitNumber);
	transmitWriteBuffer();
}

//////////////////////////////////////////////////////////////////////////
// Node Functions

/**
* Callback function
* Join the RF24 network if not already connected.
* The network coordinator holds the networking table and does not need to join.
*/
void joinNetwork(){
	if (!connectedToNetwork && UNIT_NUMBER != COORDINATOR){
		transmitJoinRequest();
	}
}

/**
* Callback function
* The node's network connection has timed out (no requests from coordinator)
* Reset the node's RF24 network membership
*/
void resetNodeNetworkConnection(){
	Log.Info(P("Timed out from network"));
	connectedToNetwork = false;
}

/**
* Send a network request to the network coordinator
*/
void transmitJoinRequest(){
	writeBuffer.reset();

	writeBuffer.write(NETWORK_JOIN_REQUEST);
	writeBuffer.write(UNIT_NUMBER);

	writeBuffer.write(PACKET_END);

	transmitWriteBuffer();
	Log.Debug(P("Attempting network join"));
}

/**
* Transmit the collected sensor data to the network coordinator
*/
void transmitDataPacket(){
	// Senpai noticed us! Restart the timeout
	timer.restartTimer(networkTimeoutTimerID);
	Log.Info(P("Data request received"));

	prepareDataPacket();
	transmitWriteBuffer();
}

/**
* Transmit the contents of the write buffer using the RF24 network
*/
void transmitWriteBuffer(){
	radio.stopListening();
	radio.write(writeBuffer.getBufferAddress(), writeBuffer.getWritePosition());
	radio.startListening();
}

/**
* Load the sensor data to the write buffer
*/
void prepareDataPacket(){
	writeBuffer.reset();
	writeBuffer.write(DATA_TRANSMIT_RESPONSE);

	writeBuffer.write(UNIT_ID_CODE);
	writeBuffer.write(UNIT_NUMBER);

	writeBuffer.write(TEMPERATURE_CODE);
	writeBuffer.writeInt(int(int(sensorData[TEMPERATURE]) * 100));

	writeBuffer.write(HUMIDITY_CODE);
	writeBuffer.write(int(sensorData[HUMIDITY]));

	writeBuffer.write(ILLUMINANCE_CODE);
	writeBuffer.writeInt(int(sensorData[ILLUMINANCE]));

	writeBuffer.write(MOTION_CODE);
	writeBuffer.write(int(sensorData[MOTION]));

	writeBuffer.write(DATA_PACKET_FINISHED);

	writeBuffer.write(PACKET_END);
}

/**
* Confirm the RF24 network has been joined so we can stop spamming the coordinator
*/
void processNetworkJoin(){
	connectedToNetwork = true;
	networkTimeoutTimerID = timer.setTimeout(NODE_TIMEOUT, resetNodeNetworkConnection);

	Log.Info(P("Joined network"));
}


//////////////////////////////////////////////////////////////////////////
// Sensors

/**
* Initialise all attached sensors
*/
void startSensors(){
	sensorData[ID] = unitID.c_str();
	sensorData["version"] = LURKER_VERSION;

	Log.Debug(P("Starting sensors..."));

	startTemperature();
	startHumidity();
	startIlluminance();
	startMotion();

	// Set up periodic sensor reads
	printDataTimerID = timer.setInterval(SAMPLE_INTERVAL, printSensorData);
}

/**
* Read all sensors
*/
void readSensors(){
	sensorData[TEMPERATURE] = readTemperature();
	sensorData[HUMIDITY] = readHumidity();
	sensorData[ILLUMINANCE] = readIlluminance();
	sensorData[MOTION] = motionDetected;

	flashSensorReadLight();
}

/**
* Start the temperature sensor
*/
void startTemperature(){
	tempSensor.begin();
	Log.Debug(P("Temperature started..."));
}

/**
* Take a temperature reading
*
* Returns:
*	Temperature reading in degrees Celsius
*/
float readTemperature(){
	tempSensor.requestTemperatures();
	return tempSensor.getTempCByIndex(0);;
}

/**
* Start the humidity sensor
*/
void startHumidity(){
	Log.Debug(P("Humidity started..."));
}

/**
* Take a humidity reading.
* May take up to 2 seconds with the DHT11 sensor
*
* Returns:
*	Relative humidity as a floating percentage
*/
float readHumidity(){
	float humidity = 0;
	int status = humiditySensor.read11(HUMIDITY_PIN);

	// Grab the data if read was successful
	if (status == DHTLIB_OK){
		humidity = humiditySensor.humidity;
	}
	else{
		Log.Error(P("Error reading humidity sensor. Error [%i]"), status);
	}

	return humidity;
}

/**
* Initialise the light sensor
*/
void startIlluminance(){
	lightSensor.begin();
	lightSensor.SetAddress(Device_Address_L);
	lightSensor.SetMode(Continuous_H_resolution_Mode);

	Log.Debug(P("Illuminance started"));
}

/**
* Take an illuminance reading
*
* Returns:
*	Illuminance in lux
*/
long readIlluminance(){
	return lightSensor.GetLightIntensity();
}

/**
* Initialise the PIR motion sensor
*/
void startMotion(){
	pinMode(MOTION_PIN, INPUT);
	motionDetected = false;

	// Enter sensor cooldown for calibration
	enterMotionCooldown();
	Log.Debug(P("Motion started"));
}

/**
* Check the PIR sensor for detected movement
* The sensor will output HIGH when motion is detected.
* Detections will hold the detection status HIGH until the cool-down has lapsed (default: 60s)
*
* Returns:
*	True if motion has been detected recently
*/
void readMotion(){
	// Check the sensor
	if (digitalRead(MOTION_PIN) == MOTION_DETECTED){
		motionDetected = true;
		flashMotionLight();
		Log.Debug(P("Motion detected"));

		enterMotionCooldown();
	}

	else{
		// No alarm; is fine
		motionDetected = false;
	}
}

/**
* Start the motion sensor cooldown to avoid multiple detections per event
* Cooldown is entered after every detection event, and for calibration.
*/
void enterMotionCooldown(){
	timer.deleteTimer(motionTimerId);
	motionTimerId = -1;

	timer.setTimeout(MOTION_COOLOFF, endMotionCooldown);
}

/**
* Callback function - Start monitoring the motion sensor again after a cooldown
*/
void endMotionCooldown(){
	motionTimerId = timer.setInterval(MOTION_CHECK_INTERVAL, readMotion);
}


//////////////////////////////////////////////////////////////////////////
// Buzzer

/**
* Initialise the passive buzzer
*/
void initialiseBuzzer(){
	pinMode(BUZZER_PIN, OUTPUT);
	buzzerOff();
	Log.Debug(P("Buzzer started..."));
}

/**
* Turn on the buzzer
*/
void buzzerOn(){
	digitalWrite(BUZZER_PIN, HIGH);
	Log.Debug(P("Buzzer On"));
}

/**
* Turn the buzzer off
*/
void buzzerOff(){
	digitalWrite(BUZZER_PIN, LOW);
	Log.Debug(P("Buzzer Off"));
}


//////////////////////////////////////////////////////////////////////////
// LEDs

/**
* Initialise the LED indicator lights
* Their default state is OFF
*/
void initialiseLights(){
	for (int i = 0; i < NUM_LEDS; i++){
		pinMode(leds[i], OUTPUT);
		switchLight(i, OFF);
	}
}

/**
* Switch the LED indicator light on or off.
* LEDs are directly supplied by the MCU pins
*
* Arguments:
*	lightPin - Pin location of the LED to be switched
*	state - boolean state of the LED, high for ON.
*/
void switchLight(int ledNum, bool state){
	if (ledNum < sizeof(leds)){
		digitalWrite(leds[ledNum], state);
	}

	Log.Debug(P("LED %i on pin %i switched to %T"), ledNum, leds[ledNum], state);
}

/**
* Briefly flash the motion sensor indicator
*/
void flashMotionLight(){
	switchLight(MOTION_LED, ON);

	timer.setTimeout(FLASH_TIME, endMotionLightFlash);

}

/**
* Callback for the flashMotionLight timer
* Turn the motion light off after the timer expires
*/
void endMotionLightFlash(){
	switchLight(MOTION_LED, OFF);
}

/**
* Flash the sensor read indicator
*/
void flashSensorReadLight(){
	switchLight(SENSOR_READ_LED, ON);

	timer.setTimeout(FLASH_TIME, endSensorReadFlash);
}

/**
* Callback for the flashSensorReadLight function
* Turns the read light off after the timer elapses
*/
void endSensorReadFlash(){
	switchLight(SENSOR_READ_LED, OFF);
}

