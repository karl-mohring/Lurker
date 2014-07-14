/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Leonardo, Platform=avr, Package=arduino
*/

#define __AVR_ATmega32u4__
#define __AVR_ATmega32U4__
#define USB_VID 0x2341
#define USB_PID 0x8036
#define USB_MANUFACTURER 
#define USB_PRODUCT "\"Arduino Leonardo\""
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
void handleIncomingPacket();
void processReceivedPacket();
void joinNetwork();
void transmitJoinRequest();
void transmitDataPacket();
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
void sendSoundNotification();
void checkMovement();
void sendMotionNotification();
int floatToInt(float num, int decimalShift);
void enableWatchdog();
void disableWatchdog();
void transmitChar(char message);

#include "c:\Program Files (x86)\Arduino\hardware\arduino\avr\variants\leonardo\pins_arduino.h" 
#include "c:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\arduino.h"
#include "E:\Dropbox\Projects\LurkerNano\LurkerNano.ino"
