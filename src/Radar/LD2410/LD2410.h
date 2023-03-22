/*
 *	An Arduino library for the Hi-Link LD2410 24Ghz FMCW radar sensor.
 *
 *  This sensor is a Frequency Modulated Continuous Wave radar, which makes it good for presence detection and its sensitivity at different ranges to both static and moving targets can be configured.
 *
 *	The code in this library is based off the manufacturer datasheet and reading of this initial piece of work for ESPHome https://github.com/rain931215/ESPHome-LD2410.
 *
 *	https://github.com/ncmreynolds/ld2410
 *
 *	Released under LGPL-2.1 see https://github.com/ncmreynolds/ld2410/LICENSE for full license
 *
 */
#ifndef LD2410_H
#define LD2410_H
#include <Arduino.h>

#define LD2410_MAX_FRAME_LENGTH 40
 //#define LD2410_DEBUG_DATA
#define LD2410_DEBUG_COMMANDS
//#define LD2410_DEBUG_PARSE

class LD2410 {

public:
	LD2410(HardwareSerial& serialPort);														//Constructor function
	~LD2410();														//Destructor function
	bool begin(bool waitForRadar = true);					//Start the ld2410
	void debug(Stream& terminalStream);											//Start debugging on a stream
	bool isConnected();
	bool read();
	bool presenceDetected();
	bool stationaryTargetDetected();
	uint16_t getStationaryTargetDistance();
	uint8_t getStationaryTargetEnergy();
	bool movingTargetDetected();
	uint16_t getMovingTargetDistance();
	uint8_t getMovingTargetEnergy();
	bool requestFirmwareVersion();									//Request the firmware version
	uint8_t firmware_major_version = 0;								//Reported major version
	uint8_t firmware_minor_version = 0;								//Reported minor version
	uint32_t firmware_bugfix_version = 0;							//Reported bugfix version (coded as hex)
	bool requestCurrentConfiguration();								//Request current configuration
	uint8_t max_gate = 0;
	uint8_t max_moving_gate = 0;
	uint8_t max_stationary_gate = 0;
	uint16_t sensor_idle_time = 0;
	uint8_t motion_sensitivity[9] = { 0,0,0,0,0,0,0,0,0 };
	uint8_t stationary_sensitivity[9] = { 0,0,0,0,0,0,0,0,0 };
	bool requestRestart();
	bool requestFactoryReset();
	bool requestStartEngineeringMode();
	bool requestEndEngineeringMode();
	bool setMaxValues(uint16_t moving, uint16_t stationary, uint16_t inactivityTimer);	//Realistically gate values are 0-8 but sent as uint16_t
	bool setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary);
protected:
private:
	HardwareSerial& serial;
	Stream* debugSerial = nullptr;									//The stream used for the debugging
	uint32_t uartTimeout = 100;								//How long to give up on receiving some useful data from the LD2410
	uint32_t uartLastPacket = 0;							//Time of the last packet from the radar
	uint32_t uartLastCommand = 0;							//Time of the last command sent to the radar
	uint8_t uartLatestAck = 0;
	bool wasLastCommandSuccessful = false;
	uint8_t dataFrame[LD2410_MAX_FRAME_LENGTH];				//Store the incoming data from the radar, to check it's in a valid format
	uint8_t dataFramePosition = 0;							//Where in the frame we are currently writing
	bool isAckFrame = false;										//Whether the incoming frame is LIKELY an ACK frame
	bool isWaitingForAck = false;									//Whether a command has just been sent
	uint8_t targetType = 0;
	uint16_t movingTargetDistance = 0;
	uint8_t movingTargetEnergy = 0;
	uint16_t stationaryTargetDistance = 0;
	uint8_t stationaryTargetEnergy = 0;
	uint8_t detectionDistance = 0;

	bool read_frame_();												//Try to read a frame from the UART
	bool parse_data_frame_();										//Is the current data frame valid?
	bool parse_command_frame_();									//Is the current command frame valid?
	void print_frame_();											//Print the frame for debugging
	void send_command_preamble_();									//Commands have the same preamble
	void send_command_postamble_();									//Commands have the same postamble
	bool enter_configuration_mode_();								//Necessary before sending any command
	bool leave_configuration_mode_();								//Will not read values without leaving command mode
};
#endif // LD2410_H
