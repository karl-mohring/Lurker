/* 
	Editor: http://www.visualmicro.com
	        arduino debugger, visual micro +, free forum and wiki
	
	Hardware: Arduino Nano w/ ATmega328, Platform=avr, Package=arduino
*/

#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define ARDUINO 101
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define __cplusplus
extern "C" void __cxa_pure_virtual() {;}

//
//
void initialiseRadio();
void initialiseSensors();
void checkRadio();
void processReceivedPacket();
void acknowledgeRequest(char c);
void toSendBuffer(char c);
void toSendBuffer(int i);
void resetSendBuffer();
char fromReceiveBuffer();
void resetReceiveBuffer();
void checkSensors();
void checkTemperature();
void checkHumidity();
void checkLight();
void checkSound();
void checkMovement();
void assembleDataPacket();
int floatToInt(float num, int decimalShift);
void sendDataPacket();

#include "c:\Program Files (x86)\Arduino\hardware\arduino\avr\variants\eightanaloginputs\pins_arduino.h" 
#include "c:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\arduino.h"
#include "E:\Dropbox\Projects\LurkerNano\LurkerNano.ino"
