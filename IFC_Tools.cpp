#include "IFC_Tools.h"
#include "IFC_Serial.h"

#include "Shared_Tools.h"
#include "SerialTransfer.h"
#include "neo6mGPS.h"




/////////////////////////////////////////////////////////////////////////////////////////
//sensor/actuator classes
Adafruit_BNO055 bno = Adafruit_BNO055(&Wire, 55, BNO055_ADDRESS_A);
LIDARLite myLidarLite(&Wire);
neo6mGPS myGPS;
SerialTransfer IFC_commandTransfer;
SerialTransfer IFC_telemetryTransfer;
Servo rudder;
Servo elevator;
Servo aileron_L;
Servo aileron_R;
Servo throttle;
/////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////////////
//IFC Class
IFC_Class myIFC;

void IFC_Class::begin()
{
	//initialize variables
	dataTimestamp_IMU = 0;
	controlInputs.limiter_enable = false;




	//initialize serial streams
	IFC_DEBUG_PORT.begin(DEBUG_PORT_BAUD);
	IFC_COMMAND_PORT.begin(COMMAND_PORT_BAUD);
	IFC_GPS_PORT.begin(GPS_PORT_BAUD);
	IFC_TELEM_PORT.begin(TELEM_PORT_BAUD);




	//wait for all serial ports to come online
	IFC_DEBUG_PORT.println(F("Initializing serial ports..."));
	IFC_DEBUG_PORT.print(F("Initializing command port at Serial")); IFC_DEBUG_PORT.print(IFC_COMMAND_PORT_NUMBER);  IFC_DEBUG_PORT.println(F("..."));
	IFC_DEBUG_PORT.print(F("Initializing GPS port at Serial"));     IFC_DEBUG_PORT.print(IFC_GPS_PORT_NUMBER);      IFC_DEBUG_PORT.println(F("..."));
	IFC_DEBUG_PORT.print(F("Initializing telemetry at Serial"));    IFC_DEBUG_PORT.print(IFC_TELEM_PORT_NUMBER);    IFC_DEBUG_PORT.println(F("..."));
	//while (!DEBUG_PORT);
	while (!IFC_COMMAND_PORT)
	{
		IFC_DEBUG_PORT.print(F("Initializing command port at Serial")); IFC_DEBUG_PORT.print(IFC_COMMAND_PORT_NUMBER);  IFC_DEBUG_PORT.println(F("..."));
		IFC_COMMAND_PORT.begin(COMMAND_PORT_BAUD);
		delay(500);
	}
	while (!IFC_GPS_PORT)
	{
		IFC_DEBUG_PORT.print(F("Initializing GPS port at Serial")); IFC_DEBUG_PORT.print(IFC_GPS_PORT_NUMBER);  IFC_DEBUG_PORT.println(F("..."));
		IFC_GPS_PORT.begin(GPS_PORT_BAUD);
		delay(500);
	}
	while (!IFC_TELEM_PORT)
	{
		IFC_DEBUG_PORT.print(F("Initializing telemetry at Serial")); IFC_DEBUG_PORT.print(IFC_TELEM_PORT_NUMBER);  IFC_DEBUG_PORT.println(F("..."));
		IFC_TELEM_PORT.begin(TELEM_PORT_BAUD);
		delay(500);
	}
	IFC_DEBUG_PORT.println(F("\tAll ports sucessfully initialized"));




	//turn on PWR LED
	IFC_DEBUG_PORT.println(F("Turning on PWR LED..."));
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	IFC_DEBUG_PORT.println(F("\tPWR LED on"));




	//set analog to digital conversion resolution (in bits)
	IFC_DEBUG_PORT.println(F("Setting ADC resolution (default - 16 bits)..."));
	analogReadResolution(ANALOG_RESOLUTION);
	IFC_DEBUG_PORT.println(F("\tResolution set"));




	//initialize AirComs
	IFC_DEBUG_PORT.println(F("Initializing AirComms..."));
	IFC_commandTransfer.begin(IFC_COMMAND_PORT);
	IFC_telemetryTransfer.begin(IFC_TELEM_PORT);
	IFC_DEBUG_PORT.println(F("\tAirComms initialized"));




	//initialize GPS
	IFC_DEBUG_PORT.println(F("Initializing GPS..."));
	myGPS.begin(IFC_GPS_PORT);
	IFC_DEBUG_PORT.println(F("\tGPS initialized"));




	//initialize the IMU and wait for it to "boot up"
	IFC_DEBUG_PORT.println(F("Initializing IMU..."));
	if (!bno.begin())
	{
		//there was a problem detecting the BNO055 ... check your connections
		IFC_DEBUG_PORT.println(F("\tOoops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
		
		//fatal error - halt code execution
		while(1);
	}
	else
	{
		delay(1000);
		bno.setExtCrystalUse(true);
		IFC_DEBUG_PORT.println(F("\tBNO055 successfully initialized"));
	}




	//initialize the LiDAR for long distance readings
	IFC_DEBUG_PORT.println(F("Initializing LiDAR altimeter..."));
	myLidarLite.begin(0, true);
	myLidarLite.configure(0);
	LiDAR_Counter = 0;
	IFC_DEBUG_PORT.println(F("\tLiDAR altimeter initialized..."));




	//initialize each individual servo to their respective center position
	IFC_DEBUG_PORT.println(F("Initializing Servos..."));
	rudder.attach(RUDDER_PIN);
	elevator.attach(ELEVATOR_PIN);
	aileron_L.attach(L_AILERON_PIN);
	aileron_R.attach(R_AILERON_PIN);

	rudder.writeMicroseconds(RUDDER_MID);
	elevator.writeMicroseconds(ELEVATOR_MID);
	aileron_L.writeMicroseconds(AILERON_MID);
	aileron_R.writeMicroseconds(AILERON_MID);
	IFC_DEBUG_PORT.println(F("\tServos initialized..."));




	//initialize the ESC
	IFC_DEBUG_PORT.println(F("Initializing ESC..."));
	throttle.attach(THROTTLE_PIN);
	throttle.write(THROTTLE_MIN);
	IFC_DEBUG_PORT.println(F("\tESC initialized..."));
}




bool IFC_Class::grabData_GPS()
{
	//if a new packet was processed, update the telemetry struct
	if (myGPS.available())
	{
		telemetry.latitude         = myGPS.lat_dd;
		telemetry.longitude        = myGPS.lon_dd;
		telemetry.UTC_year         = myGPS.utc_year;
		telemetry.UTC_month        = myGPS.utc_month;
		telemetry.UTC_day          = myGPS.utc_day;
		telemetry.UTC_hour         = myGPS.utc_hour;
		telemetry.UTC_minute       = myGPS.utc_min;
		telemetry.UTC_second       = myGPS.utc_sec;
		telemetry.speedOverGround  = myGPS.sog_knots;
		telemetry.courseOverGround = myGPS.cog_true;

		return true;
	}

	return false;
}




int IFC_Class::grabData_IMU()
{
	//get IMU data and convert to degrees
	auto vect = bno.getQuat().toEuler();
	telemetry.courseAngle = vect.x() * (180 / M_PI);
	telemetry.rollAngle   = vect.z() * (180 / M_PI);
	telemetry.pitchAngle  = vect.y() * (180 / M_PI);

	//convert Euler angles from degrees to radians - ONLY USED FOR LiDAR CORRECTION
	telemetry.convertedRoll  = vect.y();
	telemetry.convertedPitch = vect.z();

	//timestamp the new data
	dataTimestamp_IMU = millis();

	return 1;
}




int IFC_Class::grabData_LiDAR()
{
	//get altitude readings - including periodic bias correction
	if (LiDAR_Counter >= 100)
	{
		telemetry.altitude = myLidarLite.distance();
		LiDAR_Counter = 0;
	}
	else
	{
		telemetry.altitude = myLidarLite.distance(false);
		LiDAR_Counter = LiDAR_Counter + 1;
	}

	//use trig to find the triangulated elevation if the LiDAR sensor is not stabilized with a gimbal
	if (LIDAR_FIXED_MOUNT)
		telemetry.convertedAltitude = telemetry.altitude * cos(telemetry.convertedRoll) * cos(telemetry.convertedPitch);
	else
		telemetry.convertedAltitude = telemetry.altitude;

	return 1;
}




int IFC_Class::grabData_Pitot()
{
	uint16_t rawValue = analogRead(PITOT_PIN);

	telemetry.velocity = rawValue; //(2.0 / 65251.0) * (-24111.0 + sqrt(-5560435134679.0 + 163127500.0 * rawValue));

	return 1;
}




//send telemetry data to GS
void IFC_Class::sendTelem()
{
	//use timer to send commands to the plane at a fixed rate
	currentTime_Telem = millis();
	if ((currentTime_Telem - timeBench_Telem) >= REPORT_TELEM_PERIOD)
	{
		//reset timer
		timeBench_Telem += REPORT_TELEM_PERIOD;

		//update the radio's outgoing array with the propper information
		IFC_telemetryTransfer.txBuff[0]  = (uint8_t)(((int16_t)(telemetry.velocity * 100)) >> 8);
		IFC_telemetryTransfer.txBuff[1]  = (uint8_t)(((int16_t)(telemetry.velocity * 100)) & 0xFF);

		IFC_telemetryTransfer.txBuff[2]  = (uint8_t)(((int16_t)(telemetry.altitude * 100)) >> 8);
		IFC_telemetryTransfer.txBuff[3]  = (uint8_t)(((int16_t)(telemetry.altitude * 100)) & 0xFF);

		IFC_telemetryTransfer.txBuff[4]  = (uint8_t)(((int16_t)(telemetry.pitchAngle * 100)) >> 8);
		IFC_telemetryTransfer.txBuff[5]  = (uint8_t)(((int16_t)(telemetry.pitchAngle * 100)) & 0xFF);

		IFC_telemetryTransfer.txBuff[6]  = (uint8_t)(((int16_t)(telemetry.rollAngle * 100)) >> 8);
		IFC_telemetryTransfer.txBuff[7]  = (uint8_t)(((int16_t)(telemetry.rollAngle * 100)) & 0xFF);

		IFC_telemetryTransfer.txBuff[8]  = (uint8_t)(((int16_t)(telemetry.latitude * 100)) >> 8);
		IFC_telemetryTransfer.txBuff[9]  = (uint8_t)(((int16_t)(telemetry.latitude * 100)) & 0xFF);

		IFC_telemetryTransfer.txBuff[10] = (uint8_t)(((int16_t)(telemetry.longitude * 100)) >> 8);
		IFC_telemetryTransfer.txBuff[11] = (uint8_t)(((int16_t)(telemetry.longitude * 100)) & 0xFF);

		IFC_telemetryTransfer.txBuff[12] = (uint8_t)(telemetry.UTC_year >> 8);
		IFC_telemetryTransfer.txBuff[13] = (uint8_t)(telemetry.UTC_year & 0xFF);
		IFC_telemetryTransfer.txBuff[14] = telemetry.UTC_month;
		IFC_telemetryTransfer.txBuff[15] = telemetry.UTC_day;
		IFC_telemetryTransfer.txBuff[16] = telemetry.UTC_hour;
		IFC_telemetryTransfer.txBuff[17] = telemetry.UTC_minute;
		IFC_telemetryTransfer.txBuff[18] = (uint8_t)(((int16_t)(telemetry.UTC_second * 100)) >> 8);
		IFC_telemetryTransfer.txBuff[19] = (uint8_t)(((int16_t)(telemetry.UTC_second * 100)) & 0xFF);

		IFC_telemetryTransfer.txBuff[20] = (uint8_t)(((int16_t)(telemetry.speedOverGround * 100)) >> 8);
		IFC_telemetryTransfer.txBuff[21] = (uint8_t)(((int16_t)(telemetry.speedOverGround * 100)) & 0xFF);

		IFC_telemetryTransfer.txBuff[22] = (uint8_t)(((int16_t)(telemetry.courseOverGround * 100)) >> 8);
		IFC_telemetryTransfer.txBuff[23] = (uint8_t)(((int16_t)(telemetry.courseOverGround * 100)) & 0xFF);

		//send the telemetry data to GS
		IFC_telemetryTransfer.sendData(24);
	}
}




//update servo positions (use controlInputs commands)
void IFC_Class::updateServos()
{
	//update ESC speed
	throttle.write(constrain(controlInputs.throttle_command, THROTTLE_MIN, THROTTLE_MAX));

	//update servo positions
	elevator.writeMicroseconds(constrain(controlInputs.pitch_command, ELEVATOR_MIN, ELEVATOR_MAX));
	rudder.writeMicroseconds(constrain(controlInputs.yaw_command,     RUDDER_MIN,   RUDDER_MAX));
	aileron_L.writeMicroseconds(constrain(controlInputs.roll_command, AILERON_MIN,  AILERON_MAX));
	aileron_R.writeMicroseconds(constrain(controlInputs.roll_command, AILERON_MIN,  AILERON_MAX));
}




//update a single servo's position (use controlInputs commands)
void IFC_Class::updateSingleServo(byte INDEX, uint16_t value)
{
	if (INDEX == THROTTLE_INDEX)
		throttle.write(constrain(value, THROTTLE_MIN, THROTTLE_MAX));
	
	else if (INDEX == ELEVATOR_INDEX)
		elevator.writeMicroseconds(constrain(value, ELEVATOR_MIN, ELEVATOR_MAX));
	
	else if (INDEX == RUDDER_INDEX)
		rudder.writeMicroseconds(constrain(value, RUDDER_MIN, RUDDER_MAX));
	
	else if (INDEX == AILERON_INDEX)
	{
		aileron_L.writeMicroseconds(constrain(value, AILERON_MIN, AILERON_MAX));
		aileron_R.writeMicroseconds(constrain(value, AILERON_MIN, AILERON_MAX));
	}
	else
		IFC_DEBUG_PORT.println(F("Servo Not recognized"));
}




//keep the plane from pitching or rolling too much in any direction
void IFC_Class::bankPitchLimiter(bool enable, bool _linkConnected)
{
	//determine if user wants to engage the bank and pitch limiter
	if (enable)
	{
		//determine if the radio link is healthy and connected
		if (_linkConnected)
		{
			//update struct based on euler angles
			updateControlsLimiter(PITCH_AXIS);	//pitch
			updateControlsLimiter(ROLL_AXIS);	//roll
		}
		else
		{
			//use timer to send commands to the servos at a fixed rate
			currentTime_Limiter = millis();
			if ((currentTime_Limiter - timeBench_Limiter) >= REPORT_COMMANDS_PERIOD)
			{
				//reset timer
				timeBench_Limiter = currentTime_Limiter;

				//update struct based on euler angles
				updateControlsLimiter(PITCH_AXIS);	//pitch
				updateControlsLimiter(ROLL_AXIS);	//roll

				//update servo positions (use controlInputs commands)
				updateServos();
			}
		}
	}
}




//update struct based on euler angles
void IFC_Class::updateControlsLimiter(bool axis)
{
	//minimum servo command to get back to safe flight
	uint16_t minServoCommand;

	//determine if the current IMU data is old - if so, get new IMU data
	if ((millis() - dataTimestamp_IMU) >= LIMITER_PERIOD)
		grabData_IMU();

	//determine if pitch or roll should be tweaked (axis==true --> pitch, axis==false --> roll)
	if (axis == PITCH_AXIS)
	{
		//determine if the current pitch angle is too low
		if (telemetry.pitchAngle <= UNSAFE_PITCH_UP)
		{
			//determine minimum servo command to get back to safe flight
			minServoCommand = constrain(map(abs(telemetry.pitchAngle), abs(UNSAFE_PITCH_UP), abs(MAX_PITCH_UP), ELEVATOR_MID, ELEVATOR_MIN), ELEVATOR_MIN, ELEVATOR_MID);
			
			//determine if minimum servo command is less than current servo command - if so, replace current command with minimum servo command
			if (minServoCommand < controlInputs.pitch_command)
				controlInputs.pitch_command = minServoCommand;
		}
		//determine if the current pitch angle is too high
		else if (telemetry.pitchAngle >= UNSAFE_PITCH_DOWN)
		{
			//determine minimum servo command to get back to safe flight
			minServoCommand = constrain(map(abs(telemetry.pitchAngle), abs(UNSAFE_PITCH_DOWN), abs(MAX_PITCH_DOWN), ELEVATOR_MID, ELEVATOR_MAX), ELEVATOR_MID, ELEVATOR_MAX);
		
			//determine if minimum servo command is more than current servo command - if so, replace current command with minimum servo command
			if (minServoCommand > controlInputs.pitch_command)
				controlInputs.pitch_command = minServoCommand;
		}
	}
	else if (axis == ROLL_AXIS)
	{
		//determine if the current roll angle is too low
		if (telemetry.rollAngle <= UNSAFE_ROLL_R)
		{
			//determine minimum servo command to get back to safe flight
			minServoCommand = constrain(map(abs(telemetry.rollAngle), abs(UNSAFE_ROLL_R), abs(MAX_ROLL_R), AILERON_MID, AILERON_MAX), AILERON_MID, AILERON_MAX);

			//determine if minimum servo command is more than current servo command - if so, replace current command with minimum servo command
			if (minServoCommand > controlInputs.roll_command)
				controlInputs.roll_command = minServoCommand;
		}
		//determine if the current roll angle is too high
		else if (telemetry.rollAngle >= UNSAFE_ROLL_L)
		{
			//determine minimum servo command to get back to safe flight
			minServoCommand = constrain(map(abs(telemetry.rollAngle), abs(UNSAFE_ROLL_L), abs(MAX_ROLL_L), AILERON_MID, AILERON_MIN), AILERON_MIN, AILERON_MID);

			//determine if minimum servo command is less than current servo command - if so, replace current command with minimum servo command
			if (minServoCommand < controlInputs.roll_command)
				controlInputs.roll_command = minServoCommand;
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////
