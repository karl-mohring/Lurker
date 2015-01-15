#include <Arduino.h>

//////////////////////////////////////////////////////////////////////////
// Network Config

#define COORDINATOR 0
#define NETWORK_RESET_INTERVAL 300000LL	// Period before the routing table is reset and nodes need to rejoin
#define MAX_NETWORK_SIZE 10

//////////////////////////////////////////////////////////////////////////
// Unit-Specific Config
#define UNIT_NUMBER 1
String unitClass = "lurker";
String unitID = unitClass + UNIT_NUMBER;

//////////////////////////////////////////////////////////////////////////

// DS18B20 Temperature Probe
#define TEMPERATURE_PIN 7

// DHT11 Humidity Sensor
#define HUMIDITY_PIN 8
#define DHT_TYPE DHT11

// PIR Motion detector
#define MOTION_PIN 2
#define MOTION_INITIALISATION_TIME 2000 // Initialisation period in ms
#define MOTION_COOLOFF 60000	//Cool off period in ms
#define MOTION_CHECK_INTERVAL 100	// Period between motion detector checks in ms
#define MOTION_DETECTED HIGH

// Keyes mic
#define MIC_PIN A0

// Passive buzzer
#define BUZZER_PIN 5

// LEDs
#define LED1 A1
#define LED2 A2
#define ON HIGH
#define OFF LOW

// Radio
#define CE_PIN 9
#define CSN_PIN 10

#define SERIAL_BAUD 57600L
#define NETWORK_JOIN_INTERVAL 60000 // Time between network join attempts in ms

// Logging
#define LOGGER_LEVEL LOG_LEVEL_DEBUG


// Communication Pipes
#define BROADCAST_PIPE 0x90909090FFLL
#define BASE_PIPE 0x9090909000LL
const long UNIT_PIPE = BASE_PIPE + UNIT_NUMBER;