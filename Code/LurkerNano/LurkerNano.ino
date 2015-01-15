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

#define LURKER_VERSION 0.9

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
int leds[] = {LED0, LED1};


//////////////////////////////////////////////////////////////////////////
// Soft Config

// Sample Timer
SimpleTimer timer;
int joinTimerID;
int networkTimeoutTimerID;
int printDataTimerID;

// Communication
enum COMM_TAGS{
	PACKET_START = '#',
	PACKET_END = '$',
	DIVIDER = ',',
	
	NETWORK_JOIN_REQUEST = 'j',
	NETWORK_JOIN_CONFIRM = 'J',
	NETWORK_CONNECTION_RESET = 'R',
	DATA_PACKET_REQUEST = 'D',
	DATA_PACKET_RESPONSE = 'd',
	
	UNIT_ID_CODE = 'Z',
	TEMPERATURE_CODE = 'T',
	HUMIDITY_CODE = 'H',
	ILLUMINANCE_CODE = 'I',
	MOTION_CODE = 'M',
		
	BUZZER_ON_CODE = 'B',
	BUZZER_OFF_CODE = 'b',
	
	LIGHT_ON_CODE = 'L',
	LIGHT_OFF_CODE = 'l'
};
bool connectedToNetwork = false;

String received = "";
bool recording = false;

// Radio Buffer
StraightBuffer readBuffer(BUFFER_LENGTH);
StraightBuffer writeBuffer(BUFFER_LENGTH);

// Coordinator - Routing table
int routingTable[MAX_NETWORK_SIZE];



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
	
	initialiseSensors();
	initialiseBuzzer();
	initialiseRadio();
	initialiseLights();
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
	
	JsonObject<6> data;
	data["id"] = unitID.c_str();
	data["temperature"] = temperature;
	data["humidity"] = humidity;
	data["illuminance"] = illuminance;
	data["motion"] = motionDetected;
	
	Serial.print(char(PACKET_START));
	Serial.print(data);
	Serial.println(char(PACKET_END));
}


/**
* Check serial for incoming packets
*/
void checkSerial(){
	// Grab in bytes, one at a time
	if (Serial.available()){
		char c = Serial.read();
		
		// Packet start char received; start recording
		if (c == char(PACKET_START) && !recording){
			recording = true;
			received = "#";
			Log.Debug("Serial packet started...");
		}
		
		// Packet end char received; stop recording and process the packet
		else if (c == PACKET_END && recording){
			recording = false;
			received += "$";
			
			Log.Info("Serial packet received. [Length: %d]\n", received.length());
			Log.Debug("Received: %s\n", received.c_str());
			
			// Transfer string to the read buffer and do the thing
			readBuffer.reset();
			
			for (int i = 0; i < received.length(); i++){
				readBuffer.write(byte(received[i]));
			}
			
			handlePacket();
		}
		
		
		//
		else{
			if (recording && received.length() < (BUFFER_LENGTH - 1)){
				received += char(c);
				Log.Debug("Packet length: %d\n", received.length());
			}
		}
	}
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
	writeBuffer.write(PACKET_START);
	writeBuffer.write(NETWORK_JOIN_REQUEST);
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
	writeBuffer.write(PACKET_START);
	writeBuffer.write(DATA_PACKET_RESPONSE);
	
	writeBuffer.write(UNIT_ID_CODE);
	writeBuffer.write(UNIT_NUMBER);
	writeBuffer.write(DIVIDER);
	
	writeBuffer.write(TEMPERATURE_CODE);
	writeBuffer.writeInt(int(temperature*100));
	writeBuffer.write(DIVIDER);
	
	writeBuffer.write(HUMIDITY_CODE);
	writeBuffer.writeInt(int(humidity*100));
	writeBuffer.write(DIVIDER);
	
	writeBuffer.write(ILLUMINANCE_CODE);
	writeBuffer.writeInt(illuminance);
	writeBuffer.write(DIVIDER);
	
	writeBuffer.write(MOTION_CODE);
	writeBuffer.write(motionDetected);
	writeBuffer.write(DIVIDER);
	
	writeBuffer.write(PACKET_END);
}


/**
* Check the radio for any incoming packets.
*/
void checkRadio(){
	if (radio.available()){
		Log.Info("Packet received\n");
		readRadioPacket();
	}
}


/**
*
*/
void readRadioPacket(){
	radio.read(readBuffer.getBufferAddress(), BUFFER_LENGTH);
	handlePacket();
}


/**
*
*/
void handlePacket(){
	
	// First char needs to be a start byte
	byte b = readBuffer.read();
	Log.Debug("Command: %c\n", char(b));
	
	// Do the rest of the things
	if (b == PACKET_START){
		b = readBuffer.read();
		
		while (b != PACKET_END && readBuffer.getNumRemaining() > 0){
			Log.Debug("Command: %c\n", char(b));
			
			switch (b){
				
				case BUZZER_ON_CODE:{
					buzzerOn();
					break;
				}
				
				case BUZZER_OFF_CODE:{
					buzzerOff();
					break;
				}
				
				case DATA_PACKET_REQUEST:{
					transmitDataPacket();
					break;
				}
				
				case DATA_PACKET_RESPONSE:{
					processDataPacket();
					break;
				}
				
				case LIGHT_ON_CODE:{
					char lightID = readBuffer.read();
					switchLight(int(lightID), ON);
					break;
				}
				
				case LIGHT_OFF_CODE:{
					char lightID = readBuffer.read();
					switchLight(int(lightID), OFF);
					break;
				}

				case NETWORK_JOIN_REQUEST:{
					int unitNumber = readBuffer.read();
					addUnitToNetwork(unitNumber);
					break;
				}
				
				case NETWORK_JOIN_CONFIRM:{
					processNetworkJoin();
					break;
				}

				case NETWORK_CONNECTION_RESET:{
					resetNetworkConnection();
					break;
				}

				case DIVIDER:{
					break;
					
				}

				default:{
					Log.Error("Warning: unknown comm tag [%c]\n", char(b));
					break;
				}
			}
			
			b = readBuffer.read();	
		}	
	}
}


/**
* Confirm the RF24 network has been joined so we can stop spamming the coordinator
*/
void processNetworkJoin(){
	connectedToNetwork = true;
	Log.Info("Joined network\n");
}

void addUnitToNetwork(int unitNum){
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

// Init


/**
* Initialise all attached sensors
*/
void initialiseSensors(){
	Log.Debug("Initialising sensors...\n");
	
	initialiseTemperature();
	initialiseHumidity();
	initialiseIlluminaince();
	initialiseMotion();
	
	// Set up periodic sensor reads
	printDataTimerID = timer.setInterval(SAMPLE_INTERVAL, printSensorData);
}


/**
* Initialise the temperature sensor
*/
void initialiseTemperature(){
	tempSensor.begin();
	temperature = 0;
	Log.Debug("Temperature started...\n");
}


/**
* Initialise the humidity sensor
*/
void initialiseHumidity(){
	humiditySensor.begin();
	humidity = 0;
	Log.Debug("Humidity started...\n");
}


/**
* Initialise the light sensor
*/
void initialiseIlluminaince(){
	lightSensor.begin();
	lightSensor.SetAddress(Device_Address_L);
	lightSensor.SetMode(Continuous_H_resolution_Mode);
	
	Log.Debug("Illuminance started...\n");
}


/**
* Initialise the PIR motion sensor
*/
void initialiseMotion(){
	pinMode(MOTION_PIN, INPUT);
	delay(MOTION_INITIALISATION_TIME);
	motionDetected = false;
	
	timer.setInterval(MOTION_CHECK_INTERVAL, readMotion);
	Log.Debug("Motion started...\n");
}

// Read


/**
* Read all sensor data into global variables
*/
void readSensors(){
	temperature = readTemperature();
	humidity = readHumidity();
	illuminance = readIlluminance();
	
	Log.Debug("Temperature: %d\n", int((temperature)*100));
	Log.Debug("Humidity: %d\n", int((humidity*100)));
	Log.Debug("Illuminance: %d\n", illuminance);
	Log.Debug("Motion: %T\n", motionDetected);
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
* Take a humidity reading.
* May take up to 2 seconds with the DHT11 sensor
*
* Returns:
*	Relative humidity as a floating percentage
*/
float readHumidity(){
	return humiditySensor.readHumidity();
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
			
			Log.Info("Motion detected");
			switchLight(MOTION_LED, ON);
			
			}else{
			// No alarm; is fine
			motionDetected = false;
			
			switchLight(MOTION_LED, OFF);
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
