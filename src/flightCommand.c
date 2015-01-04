/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// Process Pilot Commands Defines and Variables
///////////////////////////////////////////////////////////////////////////////

float    rxCommand[12] = { 0.0f, 0.0f, 0.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f };
float	 previousRxCommand[12]  = { 0.0f, 0.0f, 0.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f };

uint8_t  commandInDetent[3]         = { true, true, true };
uint8_t  previousCommandInDetent[3] = { true, true, true };

///////////////////////////////////////////////////////////////////////////////
// Flight Mode Defines and Variables
///////////////////////////////////////////////////////////////////////////////

uint8_t flightMode = RATE;

uint8_t headingHoldEngaged     = false;

///////////////////////////////////////////////////////////////////////////////
// Arm State Variables
///////////////////////////////////////////////////////////////////////////////

semaphore_t armed          = false;
uint8_t     armingTimer    = 0;
uint8_t     disarmingTimer = 0;

///////////////////////////////////////////////////////////////////////////////
// Vertical Mode State Variables
///////////////////////////////////////////////////////////////////////////////

uint8_t  verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

uint16_t previousVertModeChState = MINCOMMAND;
uint16_t previousVertPanicChState = MINCOMMAND;

uint8_t  vertRefCmdInDetent         = true;
uint8_t  previousVertRefCmdInDetent = true;

float    verticalReferenceCommand;

///////////////////////////////////////////////////////////////////////////////
// Read Flight Commands
///////////////////////////////////////////////////////////////////////////////

void processFlightCommands(void)
{
    uint8_t channel;
    uint8_t channelsToRead = 8;

    float hdgDelta, simpleX, simpleY;

    if ( rcActive == true )
    {
		// Read receiver commands
    	if (eepromConfig.receiverType == PPM)
    		channelsToRead = eepromConfig.ppmChannels;

		for (channel = 0; channel < channelsToRead; channel++)
		{
			if (eepromConfig.receiverType == SPEKTRUM)
			    rxCommand[channel] = spektrumRead(eepromConfig.rcMap[channel]);
			else if (eepromConfig.receiverType == SBUS)
				rxCommand[channel] = sBusRead(eepromConfig.rcMap[channel]);
			else
			    rxCommand[channel] = rxRead(eepromConfig.rcMap[channel]);
		}

        rxCommand[ROLL]  -= eepromConfig.midCommand;  // Roll Range  -1000:1000
        rxCommand[PITCH] -= eepromConfig.midCommand;  // Pitch Range -1000:1000
        rxCommand[YAW]   -= eepromConfig.midCommand;  // Yaw Range   -1000:1000

		for (channel = 3; channel < channelsToRead; channel++)
			rxCommand[channel] -= eepromConfig.midCommand - MIDCOMMAND;  // Range 2000:4000
    }

    // Set past command in detent values
    for (channel = 0; channel < 3; channel++)
    	previousCommandInDetent[channel] = commandInDetent[channel];

    // Apply deadbands and set detent discretes'
    for (channel = 0; channel < 3; channel++)
    {
    	if ((rxCommand[channel] <= DEADBAND) && (rxCommand[channel] >= -DEADBAND))
        {
            rxCommand[channel] = 0;
  	        commandInDetent[channel] = true;
  	    }
        else
  	    {
  	        commandInDetent[channel] = false;
  	        if (rxCommand[channel] > 0)
  	        {
  		        rxCommand[channel] = (rxCommand[channel] - DEADBAND) * DEADBAND_SLOPE;
  	        }
  	        else
  	        {
  	            rxCommand[channel] = (rxCommand[channel] + DEADBAND) * DEADBAND_SLOPE;
  	        }
        }
    }

    ///////////////////////////////////

    // Check for low throttle
    if ( rxCommand[THROTTLE] < eepromConfig.minCheck )
    {
		// Check for disarm command ( low throttle, left yaw )
		if ( (rxCommand[YAW] < (eepromConfig.minCheck - MIDCOMMAND)) && (armed == true) )
		{
			disarmingTimer++;

			if (disarmingTimer > eepromConfig.disarmCount)
			{
				zeroPIDstates();
			    armed = false;
			    disarmingTimer = 0;
			}
		}
		else
		{
			disarmingTimer = 0;
		}

		// Check for gyro bias command ( low throttle, left yaw, aft pitch, right roll )
		if ( (rxCommand[YAW  ] < (eepromConfig.minCheck - MIDCOMMAND)) &&
		     (rxCommand[ROLL ] > (eepromConfig.maxCheck - MIDCOMMAND)) &&
		     (rxCommand[PITCH] < (eepromConfig.minCheck - MIDCOMMAND)) )
		{
			computeMPU6000RTData();
			pulseMotors(3);
		}

		// Check for arm command ( low throttle, right yaw)
		if ((rxCommand[YAW] > (eepromConfig.maxCheck - MIDCOMMAND) ) && (armed == false) && (execUp == true))
		{
			armingTimer++;

			if (armingTimer > eepromConfig.armCount)
			{
				zeroPIDstates();
				armed = true;
				armingTimer = 0;
			}
		}
		else
		{
			armingTimer = 0;
		}
	}

	///////////////////////////////////

	// Check for armed true and throttle command > minThrottle

    if ((armed == true) && (rxCommand[THROTTLE] > eepromConfig.minThrottle))
    	pidReset = false;
    else
    	pidReset = true;

	///////////////////////////////////

	// Check yaw in detent and flight mode to determine hdg hold engaged state

		if ((commandInDetent[YAW] == true) && (flightMode == ATTITUDE) && (headingHoldEngaged == false))
		{
			headingHoldEngaged = true;
		    setPIDstates(HEADING_PID,  0.0f);
	        setPIDstates(YAW_RATE_PID, 0.0f);
	        headingReference = heading.mag;
		}

		if (((commandInDetent[YAW] == false) || (flightMode != ATTITUDE)) && (headingHoldEngaged == true))
		{
		    headingHoldEngaged = false;
		}

	///////////////////////////////////

	// Vertical Mode Command Processing

	verticalReferenceCommand = rxCommand[THROTTLE] - eepromConfig.midCommand;

    // Set past altitude reference in detent value
    previousVertRefCmdInDetent = vertRefCmdInDetent;

    // Apply deadband and set detent discrete'
    if ((verticalReferenceCommand <= ALT_DEADBAND) && (verticalReferenceCommand >= -ALT_DEADBAND))
    {
        verticalReferenceCommand = 0;
  	    vertRefCmdInDetent = true;
  	}
    else
  	{
  	    vertRefCmdInDetent = false;
  	    if (verticalReferenceCommand > 0)
  	    {
  		    verticalReferenceCommand = (verticalReferenceCommand - ALT_DEADBAND) * ALT_DEADBAND_SLOPE;
  	    }
  	    else
  	    {
  	        verticalReferenceCommand = (verticalReferenceCommand + ALT_DEADBAND) * ALT_DEADBAND_SLOPE;
  	    }
    }

	///////////////////////////////////////////////
	// Need to have AUX channels update modes
	// based on change, to allow for both external
	// remote commanding from serial port and with
    // transmitter switches
    //
	// Conditions ---------------------------------
	// A switch can actuate multiple modes
	// Mode enables are defined by channel ranges
	// A mode is only enabled/disabled when a
    // channel range changes, this allows remote
    // commands via serial to be sent
	///////////////////////////////////////////////

	// For each mode, check if it's mode channel has changed,
    // then check if current mode channel is in range
    // lastly, check if mode is enabled or disabled
	int slot;
	for (slot=0; slot<MODE_SLOTS; slot++)
	{
		uint8_t modeChannel = eepromConfig.mode[slot].channel;
		// Detect of mode channel state changed
		if (fabs(previousRxCommand[modeChannel] - rxCommand[modeChannel]) > CHANGE_RANGE)
		{
			// Detect if mode channel is within defined range
			if ((rxCommand[modeChannel]>= eepromConfig.mode[slot].minChannelValue) && (rxCommand[modeChannel] < eepromConfig.mode[slot].maxChannelValue))
			{
				if (eepromConfig.mode[slot].state) // Mode is in ON state
				{
					switch (eepromConfig.mode[slot].modeType)
					{
						case MODE_ATTITUDE:
							flightMode = ATTITUDE;
							setPIDstates(ROLL_ATT_PID,  0.0f);
							setPIDstates(PITCH_ATT_PID, 0.0f);
							setPIDstates(HEADING_PID,   0.0f);
							break;
						case MODE_RATE:
							flightMode = RATE;
							setPIDstates(ROLL_RATE_PID,  0.0f);
							setPIDstates(PITCH_RATE_PID, 0.0f);
							setPIDstates(YAW_RATE_PID,   0.0f);
							break;
						case MODE_SIMPLE:
							hdgDelta = sensors.attitude500Hz[YAW] - homeData.magHeading;
							hdgDelta = standardRadianFormat(hdgDelta);
							simpleX = cosf(hdgDelta) * rxCommand[PITCH] + sinf(hdgDelta) * rxCommand[ROLL ];
							simpleY = cosf(hdgDelta) * rxCommand[ROLL ] - sinf(hdgDelta) * rxCommand[PITCH];
							rxCommand[ROLL ] = simpleY;
							rxCommand[PITCH] = simpleX;
							break;
						case MODE_AUTONAV:
							autoNavMode = MODE_AUTONAV;
							flightMode = ATTITUDE;
							verticalModeState = ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT;
							break;
						case MODE_POSITIONHOLD:
							autoNavMode = MODE_POSITIONHOLD;
							flightMode = ATTITUDE;
							verticalModeState = ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT;
							break;
						case MODE_RETURNTOHOME:
							autoNavMode = MODE_RETURNTOHOME;
							flightMode = ATTITUDE;
							verticalModeState = ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT;
							break;
						case MODE_ALTHOLD:
							// Altitude Hold is On
							if (verticalModeState == ALT_DISENGAGED_THROTTLE_ACTIVE)
							{
								verticalModeState = ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT;
								setPIDstates(HDOT_PID, 0.0f);
								setPIDstates(H_PID, 0.0f);
								altitudeHoldReference = hEstimate;
								throttleReference = rxCommand[THROTTLE];
							}
							else if (verticalModeState == ALT_DISENGAGED_THROTTLE_INACTIVE)
								verticalModeState = ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT;
							break;
						case MODE_PANIC:
							autoNavMode = MODE_NONE;
							verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;
							break;
					}
				}
				else // Mode is in OFF state
				{
					if (eepromConfig.mode[slot].modeType == MODE_ALTHOLD)
					{
						// Altitude Hold is Off
						if (verticalModeState == VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY)
						{
							verticalModeState = ALT_DISENGAGED_THROTTLE_INACTIVE;
							altitudeHoldReference = hEstimate;
						}
						else
							verticalModeState = ALT_DISENGAGED_THROTTLE_INACTIVE;
					}
				}
			}
		}
	}

	// Record previous channel values to check if changed in next iteration
	for (channel=AUX1; channel<LASTCHANNEL; channel++)
		previousRxCommand[channel] = rxCommand[channel];

    ///////////////////////////////////
    // AutoNavigation State Machine

	switch (autoNavMode)
	{
		case MODE_NONE:
			break;
		case MODE_AUTONAV:
			processAutoNavigation();
			break;
		case MODE_POSITIONHOLD:
			processPositionHold();
			break;
		case MODE_RETURNTOHOME:
			processReturnToHome();
			break;
	}

    ///////////////////////////////////
    // Vertical Mode State Machine

	switch (verticalModeState)
	{
		case ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT:
			if ((vertRefCmdInDetent == true) || eepromConfig.verticalVelocityHoldOnly)
		        verticalModeState = ALT_HOLD_AT_REFERENCE_ALTITUDE;
			break;
		case ALT_DISENGAGED_THROTTLE_ACTIVE:
			break;
		case ALT_HOLD_AT_REFERENCE_ALTITUDE:
		    if ((vertRefCmdInDetent == false) || eepromConfig.verticalVelocityHoldOnly)
		        verticalModeState = VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY;
			break;
		case VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY:
		    if ((vertRefCmdInDetent == true) && !eepromConfig.verticalVelocityHoldOnly)
		    {
				verticalModeState = ALT_HOLD_AT_REFERENCE_ALTITUDE;
				altitudeHoldReference = hEstimate;
			}
			break;
		case ALT_DISENGAGED_THROTTLE_INACTIVE: // This mode verifies throttle is at center when disengaging alt hold
			if (((rxCommand[THROTTLE] < throttleCmd + THROTTLE_WINDOW) && (rxCommand[THROTTLE] > throttleCmd - THROTTLE_WINDOW)) || eepromConfig.verticalVelocityHoldOnly)
			    verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;
	}
}

///////////////////////////////////////////////////////////////////////////////




