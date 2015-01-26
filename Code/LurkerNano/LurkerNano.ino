#include <SerialCommand.h>
#include <ArduinoCommand.h>
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
#include <DHT.h>
#include "avr/wdt.h"
#include "settings.h"

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
//	- Buzzer (passive)
//	- IR Blaster
//	- 2 LEDs
//
//////////////////////////////////////////////////////////////////////////
// Hardware Config

// Temperature
OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature tempSensor(&oneWire);
float temperature;

// Humidity
DHT humiditySensor(HUMIDITY_PIN, DHT_TYPE);
float humidity;

// Illuminance
BH1750FVI lightSensor;
long illuminance;

// Motion
bool motionDetected;
long timeOfLastMotion;


// Buzzer
RF24 radio(CE_PIN, CSN_PIN);

// Lights
int leds[3] = {LED0, LED1, LED_BUILTIN};




//////////////////////////////////////////////////////////////////////////
// Soft Config

// Sample Timer
SimpleTimer timer;
int joinTimerID;
int networkTimeoutTimerID;
int printDataTimerID;

// Communication
bool connectedToNetwork = false;

String received = "";
bool recording = false;

// Radio Buffer
StraightBuffer readBuffer(BUFFER_LENGTH);
StraightBuffer writeBuffer(BUFFER_LENGTH);

// Coordinator - Routing table
int routingTable[MAX_NETWORK_SIZE];

// Sensor data object
JsonObject<8> sensorData;

// Command handlers
ArduinoCommand commandHandler;
SerialCommand sCmd;

//////////////////////////////////////////////////////////////////////////
// Main Functions
//////////////////////////////////////////////////////////////////////////


/**
* Initial script - Run once
*/
void setup()
{
	Log.Init(LOGGER_LEVEL, SERIAL_BAUD);
	Log.Info("Lurker starting - %s\n", unitID.c_str());
	
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
	checkRadio();
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
		commandHandler.read(inChar);
	}
}

void startCommandHandler(){
	commandHandler.setTerminator(PACKET_END);
	commandHandler.setDefaultHandler(commandNotRecognised);
	
	commandHandler.addCommand(BUZZER_ON_CODE, buzzerOn);
	commandHandler.addCommand(BUZZER_OFF_CODE, buzzerOff);
	commandHandler.addCommand(NETWORK_JOIN_REQUEST, addUnitToNetwork);
	commandHandler.addCommand(NETWORK_JOIN_CONFIRM, processNetworkJoin);
	commandHandler.addCommand(DATA_TRANSMIT_REQUEST, transmitDataPacket);
	commandHandler.addCommand(SENSOR_READ_REQUEST, printSensorData);
}

void commandNotRecognised(const char *command){
	Log.Error("Warning - Unknown command\n");
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
	radio.openReadingPipe(2, BROADCAST_PIPE);
	
	// Set up network joining
	if (UNIT_NUMBER != COORDINATOR){
		timer.setInterval(NETWORK_JOIN_INTERVAL, joinNetwork);
	}
	
	Log.Debug("Radio started...\n");
}


// Periodic function - Join RF24 network
/**
* Join the RF24 network if not already connected.
* The network coordinator holds the networking table and does not need to join.
*/
void joinNetwork(){
	if (!connectedToNetwork && UNIT_NUMBER != COORDINATOR){
		transmitJoinRequest();
	}
}


// Periodic function - Reset RF24 network membership
void resetNetworkConnection(){
	connectedToNetwork = false;
}


/**
* Send a network request to the network coordinator
*/
void transmitJoinRequest(){
	writeBuffer.reset();
	writeBuffer.write(NETWORK_JOIN_REQUEST[0]);
	writeBuffer.write(UNIT_NUMBER);
	writeBuffer.write(PACKET_END);
	
	transmitWriteBuffer();
	Log.Debug("Attempting network join\n");
}


/**
* Transmit the collected sensor data to the network coordinator
*/
void transmitDataPacket(){
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
	writeBuffer.write(PACKET_START[0]);
	writeBuffer.write(DATA_TRANSMIT_RESPONSE[0]);
	
	writeBuffer.write(UNIT_ID_CODE[0]);
	writeBuffer.write(UNIT_NUMBER);
	
	writeBuffer.write(TEMPERATURE_CODE[0]);
	writeBuffer.writeInt(int(temperature*100));
	
	writeBuffer.write(HUMIDITY_CODE[0]);
	writeBuffer.writeInt(int(humidity*100));
	
	writeBuffer.write(ILLUMINANCE_CODE[0]);
	writeBuffer.writeInt(illuminance);

	writeBuffer.write(MOTION_CODE[0]);
	writeBuffer.write(motionDetected);
	
	writeBuffer.write(PACKET_END);
}


/**
* Check the radio for any incoming packets.
*/
void checkRadio(){
	if (radio.available()){
		Log.Info("Packet received\n");
		readRadioPacket();
		
		while (readBuffer.available()){
			commandHandler.read(readBuffer.read());
		}
	}
}


/**
*
*/
void readRadioPacket(){
	radio.read(readBuffer.getBufferAddress(), BUFFER_LENGTH);
	readBuffer.setWritePosition(BUFFER_LENGTH);
}


/**
* Confirm the RF24 network has been joined so we can stop spamming the coordinator
*/
void processNetworkJoin(){
	connectedToNetwork = true;
	Log.Info("Joined network\n");
}


void addUnitToNetwork(){
	char unitNum = commandHandler.next()[0];
	
	if (unitNum <= MAX_NETWORK_SIZE){
		routingTable[unitNum] = timer.setTimeout(NETWORK_RESET_INTERVAL, cleanRoutingTable);
	}
}


void processDataPacket(){
	//TODO: Process incoming data packets over RF24
}

void sendACK(){
	//TODO send an ack packet after receiving a data packet from a node
}

void processACK(){
	//TODO reset the network timeout of the node after receiving an ACK from the coordinator
}

void cleanRoutingTable(){
	for (int i = 0; i < MAX_NETWORK_SIZE; i++){
		
		// Active units have a timer ID higher than -1.
		if (routingTable[i] >= 0){
			
			// Clear any units from the routing table that are non-responsive
			if(!timer.isEnabled(routingTable[i])){
				routingTable[i] = -1;
			}
		}
	}
}

/**
* Transmit a character to the specified node
*/
void transmitChar(char message){
	radio.stopListening();
	radio.write(&message, 1);
	radio.startListening();
	Log.Verbose("Sending: %c\n", message);
}


//////////////////////////////////////////////////////////////////////////
// Sensors

// All


/**
* Initialise all attached sensors
*/
void startSensors(){
	sensorData["id"] = unitID.c_str();
	sensorData["version"] = LURKER_VERSION;
	
	Log.Debug("Starting sensors...\n");
	
	startTemperature();
	startHumidity();
	startIlluminance();
	startMotion();
	
	// Set up periodic sensor reads
	printDataTimerID = timer.setInterval(SAMPLE_INTERVAL, printSensorData);
}


/**
* Read all sensor data into global variables
*/
void readSensors(){
	sensorData["temperature"] = readTemperature();
	sensorData["humidity"] = readHumidity();
	sensorData["illuminance"] = readIlluminance();
	sensorData["motion"] = motionDetected;
	
	flashSensorReadLight();
}


// Temperature

/**
* Start the temperature sensor
*/
void startTemperature(){
	tempSensor.begin();
	temperature = 0;
	Log.Debug("Temperature started...\n");
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


// Humidity

/**
* Start the humidity sensor
*/
void startHumidity(){
	humiditySensor.begin();
	humidity = 0;
	Log.Debug("Humidity started...\n");
}


/**
* Take a humidity reading.
* May take up to 2 seconds with the DHT11 sensor
*
* Returns:
*	Relative humidity as a floating percentage
*/
float readHumidity(){
	return humiditySensor.readHumidity();
}


// Illuminance

/**
* Initialise the light sensor
*/
void startIlluminance(){
	lightSensor.begin();
	lightSensor.SetAddress(Device_Address_L);
	lightSensor.SetMode(Continuous_H_resolution_Mode);
	
	Log.Debug("Illuminance started...\n");
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


// Motion

/**
* Initialise the PIR motion sensor
*/
void startMotion(){
	pinMode(MOTION_PIN, INPUT);
	delay(MOTION_INITIALISATION_TIME);
	motionDetected = false;
	
	timer.setInterval(MOTION_CHECK_INTERVAL, readMotion);
	Log.Debug("Motion started...\n");
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
	// Only check the motion alarm if the cool-off has been exceeded
	if((millis() - timeOfLastMotion) > (MOTION_COOLOFF)){
		
		// Check the sensor
		if (digitalRead(MOTION_PIN) == MOTION_DETECTED){
			motionDetected = true;
			timeOfLastMotion = millis();
			
			Log.Info("Motion detected\n");
			flashMotionLight();
			
			}else{
			// No alarm; is fine
			motionDetected = false;
		}
	}
}


//////////////////////////////////////////////////////////////////////////
// Buzzer


/**
* Initialise the passive buzzer
*/
void initialiseBuzzer(){
	pinMode(BUZZER_PIN, OUTPUT);
	buzzerOff();
	Log.Debug("Buzzer started...\n");
}


/**
* Turn on the buzzer
*/
void buzzerOn(){
	digitalWrite(BUZZER_PIN, HIGH);
	Log.Debug("Buzzer on\n");
}


/**
* Turn the buzzer off
*/
void buzzerOff(){
	digitalWrite(BUZZER_PIN, LOW);
	Log.Debug("Buzzer off\n");
}


//////////////////////////////////////////////////////////////////////////
// LEDs


/**
* Initialise the LED indicator lights
* Their default state is OFF
*/
void initialiseLights(){
	for(int i = 0; i < sizeof(leds); i++){
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

	Log.Debug("LED%d on pin %d - %T\n", ledNum, leds[ledNum], state);
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


//////////////////////////////////////////////////////////////////////////
// Watchdog


/**
* Enable the watchdog timer
* The program will restart if it hangs for more than 8 seconds
*/
void enableWatchdog(){
	// Disable interrupts while setting up the watchdog
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
