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
RF24 radio(CE_PIN,CSN_PIN);
const long unitPipe = BASE_PIPE + UNIT_ID;


/**
* Communication Protocol
*/
enum COMM_CODES{
	TIME_UPDATE = 't',
	MOTION_EVENT = 'M',
	TIME_SINCE_MOTION_EVENT = 'm',
	IR_REQUEST = 'I',
	SOUND_EVENT = 'S',
	SOUND_LEVEL_REQUEST = 's',
	BUZZ_REQUEST = 'B',
	LED_REQUEST = 'L',
	ACKNOWLEDGE_REQUEST = '+',
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

//////////////////////////////////////////////////////////////////////////

float temperature;
float humidity;
int illuminance;
int	noiseLevel;
bool noiseTriggered;
bool movementDetected;

unsigned long timeOfLastMovement;
unsigned long timeSinceLastMovement;
int dailyMovementEvents;

#define SEND_BUFFER_SIZE 32
#define RECEIVE_BUFFER_SIZE 32
char sendBuffer[SEND_BUFFER_SIZE];
char receiveBuffer[RECEIVE_BUFFER_SIZE];
byte sendBufferPutter;
byte receiveBufferGetter;
//////////////////////////////////////////////////////////////////////////

/**
* Initialization
*/
void setup()
{
	initialiseRadio();
	initialiseSensors();
}


/**
* Main Loop
*/
void loop()
{
	checkRadio();
	checkSensors();
	assembleDataPacket();
	sendDataPacket();

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
* Check for incoming packets and act accordingly
*/
void checkRadio(){
	if (radio.available()){
		radio.read(receiveBuffer, RECEIVE_BUFFER_SIZE);
		processReceivedPacket();
	}
}


void processReceivedPacket(){
	//TODO process packets once received
	
	switch 
}


/**
* Send an acknowledgment packet to the base station
*/
void acknowledgeRequest(char c){
	radio.stopListening();
	
	radio.openWritingPipe(BASE_PIPE);
	
	
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
* Insert a 16-bit integer into the buffer
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
*/
void checkSensors(){
	checkTemperature();
	checkHumidity();
	checkLight();
	checkSound();
	checkMovement();
}


/**
* Take a temperature reading.
*/
void checkTemperature(){
	tempSensor.requestTemperatures();
	temperature = floatToInt(tempSensor.getTempCByIndex(0), 2);
}


/**
* Check the relative humidity sensor.
* Output given as a percentage (float)
* The DHT11 takes up to 2 seconds to deliver a response.
*/
void checkHumidity(){
	humidity = floatToInt(humiditySensor.readHumidity(), 2);
}


/**
* Check the light level hitting the sensor.
* Values are in lux.
*/
void checkLight(){
	illuminance = lightSensor.GetLightIntensity();
}


/**
* Check if the sound level threshold has been exceeded
*/
void checkSound(){
	//TODO check if sound level threshold has been exceeded
}


/**
* Check the PIR sensor for detected movement
* The sensor will output HIGH when motion is detected.
* Output will fall back to LOW after a set delay time (adjustable on the sensor)
* The delay timer is reset with each detection; i.e. The output will be high so long as motion is detected + the delay.
*/
void checkMovement(){
	movementDetected = digitalRead(PIR_PIN);
	
	// If motion detected, reset the movement timer
	if (movementDetected == MOVEMENT_DETECTED){
		timeSinceLastMovement = 0;
		timeOfLastMovement = millis();
	}
	else{
		timeSinceLastMovement = millis() - timeOfLastMovement;
	}
}


void assembleDataPacket(){
	toSendBuffer(TEMPERATURE_DATA);
	toSendBuffer(temperature);
	
	toSendBuffer(HUMIDITY_DATA);
	toSendBuffer(humidity);
	
	toSendBuffer(LIGHT_DATA);
	toSendBuffer(illuminance);
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

void sendDataPacket(){
	radio.stopListening();
	
	radio.openWritingPipe(BASE_PIPE);
	radio.write(sendBuffer, sendBufferPutter);
	
	radio.startListening();
}