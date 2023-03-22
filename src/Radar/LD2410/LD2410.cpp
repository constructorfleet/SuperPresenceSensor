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
#include "LD2410.h"


LD2410::LD2410(HardwareSerial& serialPort): serial(serialPort) {
	serial = serialPort;
}

LD2410::~LD2410() {
}

bool LD2410::begin(bool waitForRadar) {
	if (debugSerial != nullptr)
	{
		debugSerial->println(F("ld2410 started"));
	}
	if (waitForRadar)
	{
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nLD2410 firmware: "));
		}
		if (requestFirmwareVersion())
		{
			if (debugSerial != nullptr)
			{
				debugSerial->print(F(" v"));
				debugSerial->print(firmware_major_version);
				debugSerial->print('.');
				debugSerial->print(firmware_minor_version);
				debugSerial->print('.');
				debugSerial->print(firmware_bugfix_version);
			}
			return true;
		}
		else
		{
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("no response"));
			}
		}
	}
	else
	{
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nLD2410 library configured"));
		}
		return true;
	}
	return false;
}

void LD2410::debug(Stream& terminalStream) {
	debugSerial = &terminalStream;		//Set the stream used for the terminal
#if defined(ESP8266)
	if (&terminalStream == &Serial)
	{
		debugSerial->write(17);			//Send an XON to stop the hung terminal after reset on ESP8266
	}
#endif
}

bool LD2410::isConnected() {
	if (millis() - uartLastPacket < uartTimeout)	//Use the last reading
	{
		return true;
	}
	if (read_frame_())	//Try and read a frame if the current reading is too old
	{
		return true;
	}
	return false;
}

bool LD2410::read() {
	return read_frame_();
}

bool LD2410::presenceDetected() {
	return targetType != 0;
}

bool LD2410::stationaryTargetDetected() {
	if ((targetType & 0x02) && stationaryTargetDistance > 0 && stationaryTargetEnergy > 0)
	{
		return true;
	}
	return false;
}

uint16_t LD2410::getStationaryTargetDistance() {
	//if(stationaryTargetEnergy > 0)
	{
		return stationaryTargetDistance;
	}
	//return 0;
}

uint8_t LD2410::getStationaryTargetEnergy() {
	//if(stationaryTargetDistance > 0)
	{
		return stationaryTargetEnergy;
	}
	//return 0;
}

bool LD2410::movingTargetDetected() {
	if ((targetType & 0x01) && movingTargetDistance > 0 && movingTargetEnergy > 0)
	{
		return true;
	}
	return false;
}

uint16_t LD2410::getMovingTargetDistance() {
	//if(movingTargetEnergy > 0)
	{
		return movingTargetDistance;
	}
	//return 0;
}

uint8_t LD2410::getMovingTargetEnergy() {
	//if(movingTargetDistance > 0)
	{
		return movingTargetEnergy;
	}
	//return 0;
}

bool LD2410::read_frame_() {
	bool isDataFrameStarted = false;
	if (serial.available())
	{
		if (isDataFrameStarted == false)
		{
			uint8_t byte_read_ = serial.read();
			if (byte_read_ == 0xF4)
			{
#ifdef LD2410_DEBUG_DATA
				if (debugSerial != nullptr)
				{
					debugSerial->print(F("\nRcvd : 00 "));
				}
#endif
				dataFrame[dataFramePosition++] = byte_read_;
				isDataFrameStarted = true;
				isAckFrame = false;
			}
			else if (byte_read_ == 0xFD)
			{
#ifdef LD2410_DEBUG_COMMANDS
				if (debugSerial != nullptr)
				{
					debugSerial->print(F("\nRcvd : 00 "));
				}
#endif
				dataFrame[dataFramePosition++] = byte_read_;
				isDataFrameStarted = true;
				isAckFrame = true;
			}
		}
		else
		{
			if (dataFramePosition < LD2410_MAX_FRAME_LENGTH)
			{
#ifdef LD2410_DEBUG_DATA
				if (debugSerial != nullptr && isAckFrame == false)
				{
					if (dataFramePosition < 0x10)
					{
						debugSerial->print('0');
					}
					debugSerial->print(dataFramePosition, HEX);
					debugSerial->print(' ');
				}
#endif
#ifdef LD2410_DEBUG_COMMANDS
				if (debugSerial != nullptr && isAckFrame == true)
				{
					if (dataFramePosition < 0x10)
					{
						debugSerial->print('0');
					}
					debugSerial->print(dataFramePosition, HEX);
					debugSerial->print(' ');
				}
#endif
				dataFrame[dataFramePosition++] = serial.read();
				if (dataFramePosition > 7)	//Can check for start and end
				{
					if (dataFrame[0] == 0xF4 &&	//Data frame end state
						dataFrame[1] == 0xF3 &&
						dataFrame[2] == 0xF2 &&
						dataFrame[3] == 0xF1 &&
						dataFrame[dataFramePosition - 4] == 0xF8 &&
						dataFrame[dataFramePosition - 3] == 0xF7 &&
						dataFrame[dataFramePosition - 2] == 0xF6 &&
						dataFrame[dataFramePosition - 1] == 0xF5
						)
					{
						if (parse_data_frame_())
						{
#ifdef LD2410_DEBUG_DATA
							if (debugSerial != nullptr)
							{
								debugSerial->print(F("parsed data OK"));
							}
#endif
							isDataFrameStarted = false;
							dataFramePosition = 0;
							return true;
						}
						else
						{
#ifdef LD2410_DEBUG_DATA
							if (debugSerial != nullptr)
							{
								debugSerial->print(F("failed to parse data"));
							}
#endif
							isDataFrameStarted = false;
							dataFramePosition = 0;
						}
					}
					else if (dataFrame[0] == 0xFD &&	//Command frame end state
						dataFrame[1] == 0xFC &&
						dataFrame[2] == 0xFB &&
						dataFrame[3] == 0xFA &&
						dataFrame[dataFramePosition - 4] == 0x04 &&
						dataFrame[dataFramePosition - 3] == 0x03 &&
						dataFrame[dataFramePosition - 2] == 0x02 &&
						dataFrame[dataFramePosition - 1] == 0x01
						)
					{
						if (parse_command_frame_())
						{
#ifdef LD2410_DEBUG_COMMANDS
							if (debugSerial != nullptr)
							{
								debugSerial->print(F("parsed command OK"));
							}
#endif
							isDataFrameStarted = false;
							dataFramePosition = 0;
							return true;
						}
						else
						{
#ifdef LD2410_DEBUG_COMMANDS
							if (debugSerial != nullptr)
							{
								debugSerial->print(F("failed to parse command"));
							}
#endif
							isDataFrameStarted = false;
							dataFramePosition = 0;
						}
					}
				}
			}
			else
			{
#if defined(LD2410_DEBUG_DATA) || defined(LD2410_DEBUG_COMMANDS)
				if (debugSerial != nullptr)
				{
					debugSerial->print(F("\nLD2410 frame overran"));
				}
#endif
				isDataFrameStarted = false;
				dataFramePosition = 0;
			}
		}
	}
	return false;
}

void LD2410::print_frame_() {
	if (debugSerial != nullptr)
	{
		if (isAckFrame == true)
		{
			debugSerial->print(F("\nCmnd : "));
		}
		else
		{
			debugSerial->print(F("\nData : "));
		}
		for (uint8_t i = 0; i < dataFramePosition; i++)
		{
			if (dataFrame[i] < 0x10)
			{
				debugSerial->print('0');
			}
			debugSerial->print(dataFrame[i], HEX);
			debugSerial->print(' ');
		}
	}
}

bool LD2410::parse_data_frame_() {
	uint16_t intraDataFrameLength = dataFrame[4] + (dataFrame[5] << 8);
	if (dataFramePosition == intraDataFrameLength + 10)
	{
#ifdef LD2410_DEBUG_DATA
		if (debugSerial != nullptr && isAckFrame == false)
		{
			print_frame_();
		}
#endif
#ifdef LD2410_DEBUG_COMMANDS
		if (debugSerial != nullptr && isAckFrame == true)
		{
			print_frame_();
		}
#endif
		if (dataFrame[6] == 0x01 && dataFrame[7] == 0xAA)	//Engineering mode data
		{
			targetType = dataFrame[8];
#ifdef LD2410_DEBUG_PARSE
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("\nEngineering data - "));
				if (targetType == 0x00)
				{
					debugSerial->print(F("no target"));
				}
				else if (targetType == 0x01)
				{
					debugSerial->print(F("moving target:"));
				}
				else if (targetType == 0x02)
				{
					debugSerial->print(F("stationary target:"));
				}
				else if (targetType == 0x03)
				{
					debugSerial->print(F("moving & stationary targets:"));
				}
			}
#endif
			/*
			 *
			 *	To-do support engineering mode
			 *
			 */
		}
		else if (intraDataFrameLength == 13 && dataFrame[6] == 0x02 && dataFrame[7] == 0xAA && dataFrame[17] == 0x55 && dataFrame[18] == 0x00)	//Normal target data
		{
			targetType = dataFrame[8];
			movingTargetDistance = dataFrame[9] + (dataFrame[10] << 8);
			movingTargetEnergy = dataFrame[11];
			stationaryTargetDistance = dataFrame[12] + (dataFrame[13] << 8);
			stationaryTargetEnergy = dataFrame[14];
			detectionDistance = dataFrame[15];
#ifdef LD2410_DEBUG_PARSE
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("\nNormal data - "));
				if (targetType == 0x00)
				{
					debugSerial->print(F("no target"));
				}
				else if (targetType == 0x01)
				{
					debugSerial->print(F("moving target:"));
				}
				else if (targetType == 0x02)
				{
					debugSerial->print(F("stationary target:"));
				}
				else if (targetType == 0x03)
				{
					debugSerial->print(F("moving & stationary targets:"));
				}
				if (dataFrame[8] & 0x01)
				{
					debugSerial->print(F(" moving at "));
					debugSerial->print(movingTargetDistance);
					debugSerial->print(F("cm power "));
					debugSerial->print(movingTargetEnergy);
				}
				if (dataFrame[8] & 0x02)
				{
					debugSerial->print(F(" stationary at "));
					debugSerial->print(stationaryTargetDistance);
					debugSerial->print(F("cm power "));
					debugSerial->print(stationaryTargetEnergy);
				}
			}
#endif
			uartLastPacket = millis();
			return true;
		}
		else
		{
#ifdef LD2410_DEBUG_DATA
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("\nUnknown frame type"));
			}
#endif
			print_frame_();
		}
	}
	else
	{
#ifdef LD2410_DEBUG_DATA
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nFrame length unexpected: "));
			debugSerial->print(dataFramePosition);
			debugSerial->print(F(" not "));
			debugSerial->print(intraDataFrameLength + 10);
		}
#endif
	}
	return false;
}

bool LD2410::parse_command_frame_() {
	uint16_t intraDataFrameLength = dataFrame[4] + (dataFrame[5] << 8);
#ifdef LD2410_DEBUG_COMMANDS
	if (debugSerial != nullptr)
	{
		print_frame_();
		debugSerial->print(F("\nACK frame payload: "));
		debugSerial->print(intraDataFrameLength);
		debugSerial->print(F(" bytes"));
	}
#endif
	uartLatestAck = dataFrame[6];
	wasLastCommandSuccessful = (dataFrame[8] == 0x00 && dataFrame[9] == 0x00);
	if (intraDataFrameLength == 8 && uartLatestAck == 0xFF)
	{
#ifdef LD2410_DEBUG_COMMANDS
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nACK for entering configuration mode: "));
		}
#endif
		if (wasLastCommandSuccessful)
		{
			uartLastPacket = millis();
#ifdef LD2410_DEBUG_COMMANDS
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("OK"));
			}
#endif
			return true;
		}
		else
		{
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("failed"));
			}
			return false;
		}
	}
	else if (intraDataFrameLength == 4 && uartLatestAck == 0xFE)
	{
#ifdef LD2410_DEBUG_COMMANDS
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nACK for leaving configuration mode: "));
		}
#endif
		if (wasLastCommandSuccessful)
		{
			uartLastPacket = millis();
#ifdef LD2410_DEBUG_COMMANDS
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("OK"));
			}
#endif
			return true;
		}
		else
		{
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("failed"));
			}
			return false;
		}
	}
	else if (intraDataFrameLength == 4 && uartLatestAck == 0x60)
	{
#ifdef LD2410_DEBUG_COMMANDS
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nACK for setting max values: "));
		}
#endif
		if (wasLastCommandSuccessful)
		{
			uartLastPacket = millis();
#ifdef LD2410_DEBUG_COMMANDS
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("OK"));
			}
#endif
			return true;
		}
		else
		{
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("failed"));
			}
			return false;
		}
	}
	else if (intraDataFrameLength == 28 && uartLatestAck == 0x61)
	{
#ifdef LD2410_DEBUG_COMMANDS
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nACK for current configuration: "));
		}
#endif
		if (wasLastCommandSuccessful)
		{
			uartLastPacket = millis();
#ifdef LD2410_DEBUG_COMMANDS
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("OK"));
			}
#endif
			max_gate = dataFrame[11];
			max_moving_gate = dataFrame[12];
			max_stationary_gate = dataFrame[13];
			motion_sensitivity[0] = dataFrame[14];
			motion_sensitivity[1] = dataFrame[15];
			motion_sensitivity[2] = dataFrame[16];
			motion_sensitivity[3] = dataFrame[17];
			motion_sensitivity[4] = dataFrame[18];
			motion_sensitivity[5] = dataFrame[19];
			motion_sensitivity[6] = dataFrame[20];
			motion_sensitivity[7] = dataFrame[21];
			motion_sensitivity[8] = dataFrame[22];
			stationary_sensitivity[0] = dataFrame[23];
			stationary_sensitivity[1] = dataFrame[24];
			stationary_sensitivity[2] = dataFrame[25];
			stationary_sensitivity[3] = dataFrame[26];
			stationary_sensitivity[4] = dataFrame[27];
			stationary_sensitivity[5] = dataFrame[28];
			stationary_sensitivity[6] = dataFrame[29];
			stationary_sensitivity[7] = dataFrame[30];
			stationary_sensitivity[8] = dataFrame[31];
			sensor_idle_time = dataFrame[32];
			sensor_idle_time += (dataFrame[33] << 8);
#ifdef LD2410_DEBUG_COMMANDS
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("\nMax gate distance: "));
				debugSerial->print(max_gate);
				debugSerial->print(F("\nMax motion detecting gate distance: "));
				debugSerial->print(max_moving_gate);
				debugSerial->print(F("\nMax stationary detecting gate distance: "));
				debugSerial->print(max_stationary_gate);
				debugSerial->print(F("\nSensitivity per gate"));
				for (uint8_t i = 0; i < 9; i++)
				{
					debugSerial->print(F("\nGate "));
					debugSerial->print(i);
					debugSerial->print(F(" ("));
					debugSerial->print(i * 0.75);
					debugSerial->print('-');
					debugSerial->print((i + 1) * 0.75);
					debugSerial->print(F(" metres) Motion: "));
					debugSerial->print(motion_sensitivity[i]);
					debugSerial->print(F(" Stationary: "));
					debugSerial->print(stationary_sensitivity[i]);

				}
				debugSerial->print(F("\nSensor idle timeout: "));
				debugSerial->print(sensor_idle_time);
				debugSerial->print('s');
			}
#endif
			return true;
		}
		else
		{
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("failed"));
			}
			return false;
		}
	}
	else if (intraDataFrameLength == 4 && uartLatestAck == 0x64)
	{
#ifdef LD2410_DEBUG_COMMANDS
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nACK for setting sensitivity values: "));
		}
#endif
		if (wasLastCommandSuccessful)
		{
			uartLastPacket = millis();
#ifdef LD2410_DEBUG_COMMANDS
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("OK"));
			}
#endif
			return true;
		}
		else
		{
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("failed"));
			}
			return false;
		}
	}
	else if (intraDataFrameLength == 12 && uartLatestAck == 0xA0)
	{
#ifdef LD2410_DEBUG_COMMANDS
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nACK for firmware version: "));
		}
#endif
		if (wasLastCommandSuccessful)
		{
			firmware_major_version = dataFrame[13];
			firmware_minor_version = dataFrame[12];
			firmware_bugfix_version = dataFrame[14];
			firmware_bugfix_version += dataFrame[15] << 8;
			firmware_bugfix_version += dataFrame[16] << 16;
			firmware_bugfix_version += dataFrame[17] << 24;
			uartLastPacket = millis();
#ifdef LD2410_DEBUG_COMMANDS
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("OK"));
			}
#endif
			return true;
		}
		else
		{
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("failed"));
			}
			return false;
		}
	}
	else if (intraDataFrameLength == 4 && uartLatestAck == 0xA2)
	{
#ifdef LD2410_DEBUG_COMMANDS
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nACK for factory reset: "));
		}
#endif
		if (wasLastCommandSuccessful)
		{
			uartLastPacket = millis();
#ifdef LD2410_DEBUG_COMMANDS
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("OK"));
			}
#endif
			return true;
		}
		else
		{
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("failed"));
			}
			return false;
		}
	}
	else if (intraDataFrameLength == 4 && uartLatestAck == 0xA3)
	{
#ifdef LD2410_DEBUG_COMMANDS
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nACK for restart: "));
		}
#endif
		if (wasLastCommandSuccessful)
		{
			uartLastPacket = millis();
#ifdef LD2410_DEBUG_COMMANDS
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("OK"));
			}
#endif
			return true;
		}
		else
		{
			if (debugSerial != nullptr)
			{
				debugSerial->print(F("failed"));
			}
			return false;
		}
	}
	else
	{
#ifdef LD2410_DEBUG_COMMANDS
		if (debugSerial != nullptr)
		{
			debugSerial->print(F("\nUnknown ACK"));
		}
#endif
	}
	return false;
}

void LD2410::send_command_preamble_() {
	//Command preamble
	serial.write((byte)0xFD);
	serial.write((byte)0xFC);
	serial.write((byte)0xFB);
	serial.write((byte)0xFA);
}

void LD2410::send_command_postamble_() {
	//Command end
	serial.write((byte)0x04);
	serial.write((byte)0x03);
	serial.write((byte)0x02);
	serial.write((byte)0x01);
}

bool LD2410::enter_configuration_mode_() {
	send_command_preamble_();
	//Request firmware
	serial.write((byte)0x04);	//Command is four bytes long
	serial.write((byte)0x00);
	serial.write((byte)0xFF);	//Request enter command mode
	serial.write((byte)0x00);
	serial.write((byte)0x01);
	serial.write((byte)0x00);
	send_command_postamble_();
	uartLastCommand = millis();
	while (millis() - uartLastCommand < uartTimeout)
	{
		if (read_frame_())
		{
			if (uartLatestAck == 0xFF && wasLastCommandSuccessful)
			{
				return true;
			}
		}
	}
	return false;
}

bool LD2410::leave_configuration_mode_() {
	send_command_preamble_();
	//Request firmware
	serial.write((byte)0x02);	//Command is four bytes long
	serial.write((byte)0x00);
	serial.write((byte)0xFE);	//Request leave command mode
	serial.write((byte)0x00);
	send_command_postamble_();
	uartLastCommand = millis();
	while (millis() - uartLastCommand < uartTimeout)
	{
		if (read_frame_())
		{
			if (uartLatestAck == 0xFE && wasLastCommandSuccessful)
			{
				return true;
			}
		}
	}
	return false;
}

bool LD2410::requestStartEngineeringMode() {
	send_command_preamble_();
	//Request firmware
	serial.write((byte)0x02);	//Command is four bytes long
	serial.write((byte)0x00);
	serial.write((byte)0x62);	//Request enter command mode
	serial.write((byte)0x00);
	send_command_postamble_();
	uartLastCommand = millis();
	while (millis() - uartLastCommand < uartTimeout)
	{
		if (read_frame_())
		{
			if (uartLatestAck == 0x62 && wasLastCommandSuccessful)
			{
				return true;
			}
		}
	}
	return false;
}

bool LD2410::requestEndEngineeringMode() {
	send_command_preamble_();
	//Request firmware
	serial.write((byte)0x02);	//Command is four bytes long
	serial.write((byte)0x00);
	serial.write((byte)0x63);	//Request leave command mode
	serial.write((byte)0x00);
	send_command_postamble_();
	uartLastCommand = millis();
	while (millis() - uartLastCommand < uartTimeout)
	{
		if (read_frame_())
		{
			if (uartLatestAck == 0x63 && wasLastCommandSuccessful)
			{
				return true;
			}
		}
	}
	return false;
}

bool LD2410::requestCurrentConfiguration() {
	if (enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		//Request firmware
		serial.write((byte)0x02);	//Command is two bytes long
		serial.write((byte)0x00);
		serial.write((byte)0x61);	//Request current configuration
		serial.write((byte)0x00);
		send_command_postamble_();
		uartLastCommand = millis();
		while (millis() - uartLastCommand < uartTimeout)
		{
			if (read_frame_())
			{
				if (uartLatestAck == 0x61 && wasLastCommandSuccessful)
				{
					delay(50);
					leave_configuration_mode_();
					return true;
				}
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

bool LD2410::requestFirmwareVersion() {
	if (enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		//Request firmware
		serial.write((byte)0x02);	//Command is two bytes long
		serial.write((byte)0x00);
		serial.write((byte)0xA0);	//Request firmware version
		serial.write((byte)0x00);
		send_command_postamble_();
		uartLastCommand = millis();
		while (millis() - uartLastCommand < uartTimeout)
		{
			read_frame_();
			if (uartLatestAck == 0xA0 && wasLastCommandSuccessful)
			{
				delay(50);
				leave_configuration_mode_();
				return true;
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

bool LD2410::requestRestart() {
	if (enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		//Request firmware
		serial.write((byte)0x02);	//Command is two bytes long
		serial.write((byte)0x00);
		serial.write((byte)0xA3);	//Request restart
		serial.write((byte)0x00);
		send_command_postamble_();
		uartLastCommand = millis();
		while (millis() - uartLastCommand < uartTimeout)
		{
			if (read_frame_())
			{
				if (uartLatestAck == 0xA3 && wasLastCommandSuccessful)
				{
					delay(50);
					leave_configuration_mode_();
					return true;
				}
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

bool LD2410::requestFactoryReset() {
	if (enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		//Request firmware
		serial.write((byte)0x02);	//Command is two bytes long
		serial.write((byte)0x00);
		serial.write((byte)0xA2);	//Request factory reset
		serial.write((byte)0x00);
		send_command_postamble_();
		uartLastCommand = millis();
		while (millis() - uartLastCommand < uartTimeout)
		{
			if (read_frame_())
			{
				if (uartLatestAck == 0xA2 && wasLastCommandSuccessful)
				{
					delay(50);
					leave_configuration_mode_();
					return true;
				}
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

bool LD2410::setMaxValues(uint16_t moving, uint16_t stationary, uint16_t inactivityTimer) {
	if (enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		serial.write((byte)0x14);	//Command is 20 bytes long
		serial.write((byte)0x00);
		serial.write((byte)0x60);	//Request set max values
		serial.write((byte)0x00);
		serial.write((byte)0x00);	//Moving gate command
		serial.write((byte)0x00);
		serial.write(char(moving & 0x00FF));	//Moving gate value
		serial.write(char((moving & 0xFF00) >> 8));
		serial.write((byte)0x00);	//Spacer
		serial.write((byte)0x00);
		serial.write((byte)0x01);	//Stationary gate command
		serial.write((byte)0x00);
		serial.write(char(stationary & 0x00FF));	//Stationary gate value
		serial.write(char((stationary & 0xFF00) >> 8));
		serial.write((byte)0x00);	//Spacer
		serial.write((byte)0x00);
		serial.write((byte)0x02);	//Inactivity timer command
		serial.write((byte)0x00);
		serial.write(char(inactivityTimer & 0x00FF));	//Inactivity timer
		serial.write(char((inactivityTimer & 0xFF00) >> 8));
		serial.write((byte)0x00);	//Spacer
		serial.write((byte)0x00);
		send_command_postamble_();
		uartLastCommand = millis();
		while (millis() - uartLastCommand < uartTimeout)
		{
			if (read_frame_())
			{
				if (uartLatestAck == 0x60 && wasLastCommandSuccessful)
				{
					delay(50);
					leave_configuration_mode_();
					return true;
				}
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}

bool LD2410::setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary) {
	if (enter_configuration_mode_())
	{
		delay(50);
		send_command_preamble_();
		serial.write((byte)0x14);	//Command is 20 bytes long
		serial.write((byte)0x00);
		serial.write((byte)0x64);	//Request set sensitivity values
		serial.write((byte)0x00);
		serial.write((byte)0x00);	//Gate command
		serial.write((byte)0x00);
		serial.write(char(gate));	//Gate value
		serial.write((byte)0x00);
		serial.write((byte)0x00);	//Spacer
		serial.write((byte)0x00);
		serial.write((byte)0x01);	//Motion sensitivity command
		serial.write((byte)0x00);
		serial.write(char(moving));	//Motion sensitivity value
		serial.write((byte)0x00);
		serial.write((byte)0x00);	//Spacer
		serial.write((byte)0x00);
		serial.write((byte)0x02);	//Stationary sensitivity command
		serial.write((byte)0x00);
		serial.write(char(stationary));	//Stationary sensitivity value
		serial.write((byte)0x00);
		serial.write((byte)0x00);	//Spacer
		serial.write((byte)0x00);
		send_command_postamble_();
		uartLastCommand = millis();
		while (millis() - uartLastCommand < uartTimeout)
		{
			if (read_frame_())
			{
				if (uartLatestAck == 0x64 && wasLastCommandSuccessful)
				{
					delay(50);
					leave_configuration_mode_();
					return true;
				}
			}
		}
	}
	delay(50);
	leave_configuration_mode_();
	return false;
}