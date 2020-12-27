#include "GS_Tools.h"
#include "GS_Serial.h"
#include "Shared_Tools.h"
#include "SerialTransfer.h"




#if USE_GS_TELEM
void GS_Class::sendTelem()
{
	if (telemTimer.fire())
	{
		uint16_t sendLen = telemetryTransfer.txObj(telemetry);
		sendLen = telemetryTransfer.txObj(controlInputs, sendLen);
		sendLen += TELEMETRY_BUFFER;

		telemetryTransfer.sendData(sendLen);
	}
}
#endif




void GS_Class::begin()
{
	//initialize variables
	controlInputs.limiter_enable        = false;
	controlInputs.manual_control_enable = true;




	//initialize serial streams	
	GS_COMMAND_PORT.begin(COMMAND_PORT_BAUD);

#if USE_GS_DEBUG
	GS_DEBUG_PORT.begin(DEBUG_PORT_BAUD);
#endif
	
#if USE_GS_TELEM
	GS_TELEM_PORT.begin(TELEM_PORT_BAUD);
#endif




	//wait for all serial ports to come online
#if USE_GS_DEBUG
	GS_DEBUG_PORT.println(F("Initializing serial ports..."));
	GS_DEBUG_PORT.print(F("Initializing command port at Serial")); GS_DEBUG_PORT.print(GS_COMMAND_PORT_NUMBER); GS_DEBUG_PORT.println(F("..."));

#if USE_GS_DEBUG
	GS_DEBUG_PORT.print(F("Initializing telemetry at Serial")); GS_DEBUG_PORT.print(GS_TELEM_PORT_NUMBER); GS_DEBUG_PORT.println(F("..."));
#endif
#endif

	while (!GS_COMMAND_PORT)
	{
#if USE_GS_DEBUG
		GS_DEBUG_PORT.println(F("Initializing command port..."));
#endif
		GS_COMMAND_PORT.begin(COMMAND_PORT_BAUD);
		delay(500);
	}

#if USE_GS_TELEM
	while (!GS_TELEM_PORT)
	{
#if USE_GS_DEBUG
		GS_DEBUG_PORT.println(F("Initializing telemetry port..."));
#endif

		GS_TELEM_PORT.begin(TELEM_PORT_BAUD);
		delay(500);
	}
#endif

#if USE_GS_DEBUG
	GS_DEBUG_PORT.println(F("\tAll ports sucessfully initialized"));
#endif




	//turn on PWR LED
#if USE_GS_DEBUG
	GS_DEBUG_PORT.println(F("Turning on PWR LED..."));
#endif

	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

#if USE_GS_DEBUG
	GS_DEBUG_PORT.println(F("\tPWR LED on"));
#endif




	//set analog to digital conversion resolution (in bits)
#if USE_GS_DEBUG
	GS_DEBUG_PORT.println(F("Setting ADC resolution (default - 16 bits)..."));
#endif

	analogReadResolution(ANALOG_RESOLUTION);

#if USE_GS_DEBUG
	GS_DEBUG_PORT.println(F("\tResolution set"));
#endif




	//initialize transfer classes
#if USE_GS_DEBUG
	GS_DEBUG_PORT.println(F("Initializing transfer classes..."));
#endif

	commandTransfer.begin(GS_COMMAND_PORT);

#if USE_GS_TELEM
	telemetryTransfer.begin(GS_TELEM_PORT);
#endif

#if USE_GS_DEBUG
	GS_DEBUG_PORT.println(F("\tTansfer classes initialized"));
#endif




	//initialize "pass-through" timers
#if USE_GS_DEBUG
	GS_DEBUG_PORT.println(F("Initializing timers..."));
#endif

	commandTimer.begin(REPORT_COMMANDS_PERIOD);
	lossLinkTimer.begin(LOSS_LINK_TIMEOUT);
	telemTimer.begin(REPORT_TELEM_PERIOD);

#if USE_GS_DEBUG
	GS_DEBUG_PORT.println(F("\tTimers initialized..."));
#endif
	
	
	
	
#if USE_GS_DEBUG
	GS_DEBUG_PORT.println(F("Initialization complete"));
	GS_DEBUG_PORT.println(F("--------------------------------------------------"));
#endif
}




bool GS_Class::tick()
{
	computeAndSendCommands();

#if USE_GS_TELEM
	if (telemetryTransfer.available())
	{
		uint16_t recLen = telemetryTransfer.rxObj(telemetry);
		telemetryTransfer.rxObj(controlInputs, recLen);

		lossLinkTimer.start();
		linkConnected = true;
		telemetry.validFlags = telemetry.validFlags | 0x2;
	}
#endif

	return linkFailover();
}




//determine each command value based off GS sensor data and send commands to plane
void GS_Class::computeAndSendCommands()
{
	//use timer to send commands to the plane at a fixed rate
	if (commandTimer.fire())
	{
		computeCommands();
		sendCommands();
	}
}




//determine each command value based off GS sensor data
void GS_Class::computeCommands()
{
	//read and map joystick data
	//mapping normalized to max and min joystick values
	controlInputs.roll_command     = updateServoCommand(ailerons);
	controlInputs.pitch_command    = updateServoCommand(elevator);
	controlInputs.yaw_command      = updateServoCommand(rudder);
	controlInputs.throttle_command = updateServoCommand(throttle);
}




//send telemetry data to GS
void GS_Class::sendCommands()
{
	//update the radio's outgoing array with the propper information
	commandTransfer.txObj(controlInputs);
	commandTransfer.sendData(sizeof(controlInputs) + COMMAND_BUFFER); //allow extra bytes to send (bytes are user defined)
}




int16_t GS_Class::updateServoCommand(const control_surfaces_struct& controlSurface)
{
	//value to be sent to plane
	uint16_t commandValue = 0;

	//read the analog voltage of the potentiometer
	uint16_t analogValue = analogRead(controlSurface.analog_pin);

	if (controlSurface.reverse)
	{
		//convert to a servo command format
		commandValue = map(analogValue,                //value to be mapped
		                   controlSurface.ADC_min,     //input minimum expected value
			               controlSurface.ADC_max,     //input maximum expected value
		                   controlSurface.surface_max, //output maximum value
		                   controlSurface.surface_min  //output minimum value
		               ) + controlSurface._offset;     //add the offset (bias)
	}
	else
	{
		//convert to a servo command format
		commandValue = map(analogValue,                //value to be mapped
			               controlSurface.ADC_min,     //input minimum expected value
			               controlSurface.ADC_max,     //input maximum expected value
			               controlSurface.surface_min, //output minimum value
			               controlSurface.surface_max  //output maximum value
		                   ) + controlSurface._offset; //add the offset (bias)
	}

	commandValue = constrain(commandValue,                //value to be constrained
		                     controlSurface.surface_min,  //minimum value
		                     controlSurface.surface_max); //maximum value

	return commandValue;
}




bool GS_Class::linkFailover()
{
	if (lossLinkTimer.fire(false))
	{
		//do something

		linkConnected = false;
		telemetry.validFlags = telemetry.validFlags & !0x2;
	}

	return linkConnected;
}
