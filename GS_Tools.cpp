#include "GS_Tools.h"
#include "GS_Serial.h"

#include "Shared_Tools.h"
#include "SerialTransfer.h"




/////////////////////////////////////////////////////////////////////////////////////////
//radio classes
SerialTransfer GS_commandTransfer;
SerialTransfer GS_telemetryTransfer;
/////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////////////
//GS Class
GS_Class myGS;

void GS_Class::begin()
{
	//initialize serial streams	
	GS_DEBUG_PORT.begin(DEBUG_PORT_BAUD);
	GS_COMMAND_PORT.begin(COMMAND_PORT_BAUD);
	GS_TELEM_PORT.begin(TELEM_PORT_BAUD);




	//wait for all serial ports to come online
	//while (!GS_DEBUG_PORT);
	GS_DEBUG_PORT.println(F("Initializing serial ports..."));
	GS_DEBUG_PORT.print(F("Initializing command port at Serial")); GS_DEBUG_PORT.print(GS_COMMAND_PORT_NUMBER); GS_DEBUG_PORT.println(F("..."));
	while (!GS_COMMAND_PORT)
	{
		GS_DEBUG_PORT.println(F("Initializing command port..."));
		GS_COMMAND_PORT.begin(COMMAND_PORT_BAUD);
		delay(500);
	}
	GS_DEBUG_PORT.print(F("Initializing telemetry at Serial")); GS_DEBUG_PORT.print(GS_TELEM_PORT_NUMBER); GS_DEBUG_PORT.println(F("..."));
	while (!GS_TELEM_PORT)
	{
		GS_DEBUG_PORT.println(F("Initializing telemetry port..."));
		GS_TELEM_PORT.begin(TELEM_PORT_BAUD);
		delay(500);
	}
	GS_DEBUG_PORT.println(F("\tAll ports sucessfully initialized"));




	//turn on PWR LED
	GS_DEBUG_PORT.println(F("Turning on PWR LED..."));
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	GS_DEBUG_PORT.println(F("\tPWR LED on"));




	//set analog to digital conversion resolution (in bits)
	GS_DEBUG_PORT.println(F("Setting ADC resolution (default - 16 bits)..."));
	analogReadResolution(ANALOG_RESOLUTION);
	GS_DEBUG_PORT.println(F("\tResolution set"));




	//initialize AirComs
	GS_DEBUG_PORT.println(F("Initializing AirComms..."));
	GS_commandTransfer.begin(GS_COMMAND_PORT);
	GS_telemetryTransfer.begin(GS_TELEM_PORT);
	GS_DEBUG_PORT.println(F("\tAirComms initialized"));




	//initialize "pass-through" timers
	GS_DEBUG_PORT.println(F("Initializing timers..."));
	timeBench_Commands = millis();
	currentTime_Commands = timeBench_Commands;

	timeBench_Telem = millis();
	currentTime_Telem = timeBench_Telem;
	GS_DEBUG_PORT.println(F("\tTimers initialized..."));
}




//send data to datalogging computer via debugging port
void GS_Class::sendTelem()
{
	if (newTelem)
	{
		newTelem = false;

		GS_DEBUG_PORT.println("New Telem:");
		GS_DEBUG_PORT.print("Alt: ");   GS_DEBUG_PORT.println(telemetry.altitude, 5);
		GS_DEBUG_PORT.print("Roll: ");  GS_DEBUG_PORT.println(telemetry.rollAngle, 5);
		GS_DEBUG_PORT.print("Pitch: "); GS_DEBUG_PORT.println(telemetry.pitchAngle, 5);
		GS_DEBUG_PORT.print("Vel: ");   GS_DEBUG_PORT.println(telemetry.velocity, 5);
		GS_DEBUG_PORT.print("Lat: ");   GS_DEBUG_PORT.println(telemetry.latitude, 5);
		GS_DEBUG_PORT.print("Lon: ");   GS_DEBUG_PORT.println(telemetry.longitude, 5);
		GS_DEBUG_PORT.print("UTC_y: "); GS_DEBUG_PORT.println(telemetry.UTC_year);
		GS_DEBUG_PORT.print("UTC_M: "); GS_DEBUG_PORT.println(telemetry.UTC_month);
		GS_DEBUG_PORT.print("UTC_d: "); GS_DEBUG_PORT.println(telemetry.UTC_day);
		GS_DEBUG_PORT.print("UTC_h: "); GS_DEBUG_PORT.println(telemetry.UTC_hour);
		GS_DEBUG_PORT.print("UTC_m: "); GS_DEBUG_PORT.println(telemetry.UTC_minute);
		GS_DEBUG_PORT.print("UTC_s: "); GS_DEBUG_PORT.println(telemetry.UTC_second);
		GS_DEBUG_PORT.print("SOG: ");   GS_DEBUG_PORT.println(telemetry.speedOverGround, 3);
		GS_DEBUG_PORT.print("COG: ");   GS_DEBUG_PORT.println(telemetry.courseOverGround, 3);
		GS_DEBUG_PORT.println();
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
	GS_commandTransfer.txBuff[PITCH_COMMAND_MSB]     = (myGS.controlInputs.pitch_command >> 8) & 0xFF;
	GS_commandTransfer.txBuff[PITCH_COMMAND_LSB]     = (myGS.controlInputs.pitch_command) & 0xFF;
	GS_commandTransfer.txBuff[ROLL_COMMAND_MSB]      = (myGS.controlInputs.roll_command >> 8) & 0xFF;
	GS_commandTransfer.txBuff[ROLL_COMMAND_LSB]      = (myGS.controlInputs.roll_command) & 0xFF;
	GS_commandTransfer.txBuff[YAW_COMMAND_MSB]       = (myGS.controlInputs.yaw_command >> 8) & 0xFF;
	GS_commandTransfer.txBuff[YAW_COMMAND_LSB]       = (myGS.controlInputs.yaw_command) & 0xFF;
	GS_commandTransfer.txBuff[THROTTLE_COMMAND_MSB]  = (myGS.controlInputs.throttle_command >> 8) & 0xFF;
	GS_commandTransfer.txBuff[THROTTLE_COMMAND_LSB]  = (myGS.controlInputs.throttle_command) & 0xFF;
	GS_commandTransfer.txBuff[AUTOPILOT_COMMAND_MSB] = (myGS.controlInputs.autopilot_command >> 8) & 0xFF;
	GS_commandTransfer.txBuff[AUTOPILOT_COMMAND_LSB] = (myGS.controlInputs.autopilot_command) & 0xFF;
	GS_commandTransfer.txBuff[LIMITER_COMMAND_MSB]   = (myGS.controlInputs.limiter_command >> 8) & 0xFF;
	GS_commandTransfer.txBuff[LIMITER_COMMAND_LSB]   = (myGS.controlInputs.limiter_command) & 0xFF;
	GS_commandTransfer.txBuff[GEAR_COMMAND_MSB]      = (myGS.controlInputs.gear_command >> 8) & 0xFF;
	GS_commandTransfer.txBuff[GEAR_COMMAND_LSB]      = (myGS.controlInputs.gear_command) & 0xFF;
	GS_commandTransfer.txBuff[FLAPS_COMMAND_MSB]     = (myGS.controlInputs.flaps_command >> 8) & 0xFF;
	GS_commandTransfer.txBuff[FLAPS_COMMAND_LSB]     = (myGS.controlInputs.flaps_command ) & 0xFF;

	//send the telemetry data to GS - First 16 bytes are reserved, leaving 6 for user defined bytes
	GS_commandTransfer.sendData(16);
}




//determine each command value based off GS sensor data and send commands to plane
void GS_Class::computeAndSendCommands()
{
	//use timer to send commands to the plane at a fixed rate
	currentTime_Commands = millis();
	if ((currentTime_Commands - timeBench_Commands) >= REPORT_COMMANDS_PERIOD)
	{
		//reset timer
		timeBench_Commands += REPORT_COMMANDS_PERIOD;

		computeCommands();
		sendCommands();
	}
}




//check to see if there is a loss of radio link between GS and IFC
bool GS_Class::checkRadioLink()
{
	return linkConnected;
}




int16_t GS_Class::updateServoCommand(controlSurfaces controlSurface)
{
	//value to be sent to plane
	uint16_t commandValue = 0;

	//read the analog voltage of the potentiometer
	uint16_t analogValue = analogRead(controlSurface.analog_pin);

	if (controlSurface.reverse)
	{
		//convert to a servo command format
		commandValue = map(analogValue,                           //value to be mapped
		                   controlSurface.ADC_min,                //input minimum expected value
			               controlSurface.ADC_max,                //input maximum expected value
		                   controlSurface.high_rates_surface_max, //output maximum value
		                   controlSurface.high_rates_surface_min  //output minimum value
		               ) + controlSurface._offset;                //add the offset (bias)
	}
	else
	{
		//convert to a servo command format
		commandValue = map(analogValue,                           //value to be mapped
			               controlSurface.ADC_min,                //input minimum expected value
			               controlSurface.ADC_max,                //input maximum expected value
			               controlSurface.high_rates_surface_min, //output minimum value
			               controlSurface.high_rates_surface_max  //output maximum value
		                   ) + controlSurface._offset;            //add the offset (bias)
	}

	commandValue = constrain(commandValue,                           //value to be constrained
		                     controlSurface.high_rates_surface_min,	 //minimum value
		                     controlSurface.high_rates_surface_max); //maximum value

	return commandValue;
}
/////////////////////////////////////////////////////////////////////////////////////////
