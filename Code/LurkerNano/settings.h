#include <Arduino.h>

//////////////////////////////////////////////////////////////////////////
// Network Config

const byte COORDINATOR = 0;
const long NETWORK_RESET_INTERVAL = 300000;	// Period before the routing table is reset and nodes need to rejoin
const byte MAX_NETWORK_SIZE = 10;

//////////////////////////////////////////////////////////////////////////
// Unit-Specific Config
const byte UNIT_NUMBER = 2;
String unitClass = "lurker";
String unitID = unitClass + UNIT_NUMBER;

//////////////////////////////////////////////////////////////////////////

const long SAMPLE_INTERVAL = 20000;	// Sample interval in ms

// DS18B20 Temperature Probe
const byte TEMPERATURE_PIN = 7;

// DHT11 Humidity Sensor
const byte HUMIDITY_PIN = 8;
const byte DHT_TYPE = DHT11;

// PIR Motion detector
const byte MOTION_PIN = 2;
const int MOTION_INITIALISATION_TIME = 2000; // Initialisation period in ms
const long MOTION_COOLOFF = SAMPLE_INTERVAL;	//Cool off period in ms
const int MOTION_CHECK_INTERVAL = 100;	// Period between motion detector checks in ms
const byte MOTION_DETECTED = HIGH;

// Passive buzzer
const byte BUZZER_PIN = 5;

// LEDs
const byte LED0 = A1;
const byte LED1 = A2;
const byte ON = HIGH;
const byte OFF = LOW;
const byte MOTION_LED = 1;
const byte SENSOR_READ_LED = 0;
const long FLASH_TIME = 5000;

// Radio
const byte CE_PIN = 9;
const byte CSN_PIN = 10;

const long SERIAL_BAUD = 57600;
const long NETWORK_JOIN_INTERVAL = 60000; // Time between network join attempts in ms

// Logging
const int LOGGER_LEVEL = LOG_LEVEL_INFOS;


// Communication Pipes
const long BROADCAST_PIPE = 0x90909090FFLL;
const long BASE_PIPE = 0x9090909000LL;
const long UNIT_PIPE = BASE_PIPE + UNIT_NUMBER;

// Comm Tags
const char PACKET_START[] = "#",
const char PACKET_END[] = "$",
const char DIVIDER[] = ",",

const char NETWORK_JOIN_REQUEST[] = "j",
const char NETWORK_JOIN_CONFIRM[] = "J",
const char NETWORK_CONNECTION_RESET[] = "R",
const char DATA_PACKET_REQUEST[] = "D",
const char DATA_PACKET_RESPONSE[] = "d",

const char UNIT_ID_CODE[] = "Z",
const char TEMPERATURE_CODE[] = "T",
const char HUMIDITY_CODE[] = "H",
const char ILLUMINANCE_CODE[] = "I",
const char MOTION_CODE[] = "M",

const char BUZZER_ON_CODE[] = "B",
const char BUZZER_OFF_CODE[] = "b",

const char LIGHT_ON_CODE[] = "L",
const char LIGHT_OFF_CODE[] = "l"