#include "IFC_Tools.h"
#include "IFC_Serial.h"

#include "Shared_Tools.h"
#include "SerialTransfer.h"
#include "neo6mGPS.h"




/////////////////////////////////////////////////////////////////////////////////////////
//sensor/actuator classes
Adafruit_BNO055 bno = Adafruit_BNO055(&Wire, 55, BNO055_ADDRESS_A);
neo6mGPS myGPS;
SerialTransfer IFC_commandTransfer;
SerialTransfer IFC_telemetryTransfer;
SerialTransfer IFC_lidarTransfer;
Servo rudder;
Servo elevator;
Servo aileron;
Servo throttle;
/////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////////////
//IFC Class
IFC_Class myIFC;

void IFC_Class::begin()
{
	//initialize variables
	controlInputs.limiter_enable        = false;
	controlInputs.manual_control_enable = true;




	//initialize serial streams
	IFC_DEBUG_PORT.begin(DEBUG_PORT_BAUD);
	IFC_COMMAND_PORT.begin(COMMAND_PORT_BAUD);
	IFC_GPS_PORT.begin(GPS_PORT_BAUD);
	IFC_LIDAR_PORT.begin(LIDAR_PORT_BAUD);
	IFC_TELEM_PORT.begin(TELEM_PORT_BAUD);




	//wait for all serial ports to come online
	IFC_DEBUG_PORT.println(F("Initializing serial ports..."));
	IFC_DEBUG_PORT.print(F("Initializing command port at Serial")); IFC_DEBUG_PORT.print(IFC_COMMAND_PORT_NUMBER);  IFC_DEBUG_PORT.println(F("..."));
	IFC_DEBUG_PORT.print(F("Initializing GPS port at Serial"));     IFC_DEBUG_PORT.print(IFC_GPS_PORT_NUMBER);      IFC_DEBUG_PORT.println(F("..."));
	IFC_DEBUG_PORT.print(F("Initializing LiDAR port at Serial"));   IFC_DEBUG_PORT.print(IFC_LIDAR_PORT_NUMBER);    IFC_DEBUG_PORT.println(F("..."));
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
	while (!IFC_LIDAR_PORT)
	{
		IFC_DEBUG_PORT.print(F("Initializing LiDAR port at Serial")); IFC_DEBUG_PORT.print(IFC_LIDAR_PORT);  IFC_DEBUG_PORT.println(F("..."));
		IFC_LIDAR_PORT.begin(LIDAR_PORT_BAUD);
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
	IFC_DEBUG_PORT.println(F("Initializing transfer classes..."));
	IFC_commandTransfer.begin(IFC_COMMAND_PORT);
	IFC_telemetryTransfer.begin(IFC_TELEM_PORT);
	IFC_lidarTransfer.begin(IFC_LIDAR_PORT);
	IFC_DEBUG_PORT.println(F("\tTransfer classes initialized"));




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




	//initialize each individual servo to their respective center position
	IFC_DEBUG_PORT.println(F("Initializing servos..."));
	rudder.attach(RUDDER_PIN);
	elevator.attach(ELEVATOR_PIN);
	aileron.attach(AILERON_PIN);

	rudder.writeMicroseconds(RUDDER_MID);
	elevator.writeMicroseconds(ELEVATOR_MID);
	aileron.writeMicroseconds(AILERON_MID);
	IFC_DEBUG_PORT.println(F("\tServos initialized..."));




	//initialize the ESC
	IFC_DEBUG_PORT.println(F("Initializing ESC..."));
	throttle.attach(THROTTLE_PIN);
	throttle.write(THROTTLE_MIN);
	IFC_DEBUG_PORT.println(F("\tESC initialized..."));




	//initialize the timers
	IFC_DEBUG_PORT.println(F("Initializing timers..."));
	lossLinkTimer.begin(LOSS_LINK_TIMEOUT);
	limiterTimer.begin(LIMITER_PERIOD);
	telemTimer.begin(REPORT_TELEM_PERIOD);
	imuTimer.begin(LIMITER_PERIOD);
	IFC_DEBUG_PORT.println(F("\tTimers initialized..."));




	IFC_DEBUG_PORT.println(F("Initialization complete"));
	IFC_DEBUG_PORT.println(F("--------------------------------------------------"));
}




void IFC_Class::handleSerialEvents()
{
	commEvent_IFC();
	lidarEvent_IFC();
}




void IFC_Class::commEvent_IFC()
{
	if (IFC_commandTransfer.available())
	{
		packetDetected = true;

		//update controlInputs struct so that the next time the servos can be updated with the latest positions
		IFC_commandTransfer.rxObj(myIFC.controlInputs, sizeof(myIFC.controlInputs));

		lossLinkTimer.reset();
	}
	else if (IFC_commandTransfer.status < 0)
	{
		IFC_DEBUG_PORT.print("Command Link Serial Transfer ERROR: ");
		IFC_DEBUG_PORT.println(IFC_commandTransfer.status);
	}

	if (lossLinkTimer.fire(false))
	{

	}
}




void IFC_Class::lidarEvent_IFC()
{
	if (IFC_lidarTransfer.available())
	{
		//update controlInputs struct so that the next time the servos can be updated with the latest positions
		IFC_lidarTransfer.rxObj(myIFC.telemetry.altitude, sizeof(myIFC.telemetry.altitude));

		//use trig to find the triangulated elevation if the LiDAR sensor is not stabilized with a gimbal
		if (LIDAR_FIXED_MOUNT)
			myIFC.telemetry.convertedAltitude = myIFC.telemetry.altitude * cos(myIFC.telemetry.convertedRoll) * cos(myIFC.telemetry.convertedPitch);
		else
			myIFC.telemetry.convertedAltitude = myIFC.telemetry.altitude;
	}
	else if (IFC_lidarTransfer.status < 0)
	{
		IFC_DEBUG_PORT.print("LiDAR Link Serial Transfer ERROR: ");
		IFC_DEBUG_PORT.println(IFC_lidarTransfer.status);
	}
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
	telemetry.pitchAngle  = vect.z() * (180 / M_PI);
	telemetry.rollAngle   = vect.y() * (180 / M_PI);

	//convert Euler angles from degrees to radians - ONLY USED FOR LiDAR CORRECTION
	telemetry.convertedPitch = vect.z();
	telemetry.convertedRoll  = vect.y();

	//timestamp the new data - regardless of where this function was called
	imuTimer.start();

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
	if (telemTimer.fire())
	{
		//send the telemetry data to GS
		uint16_t sendLen;

		IFC_telemetryTransfer.txObj(telemetry, sizeof(telemetry));
		sendLen = sizeof(telemetry);
		
		IFC_telemetryTransfer.txObj(controlInputs, sizeof(controlInputs), sendLen);
		sendLen += sizeof(controlInputs);
		sendLen += TELEMETRY_BUFFER;

		IFC_telemetryTransfer.sendData(sendLen);
	}
}




//update servo positions (use controlInputs commands)
void IFC_Class::updateServos(bool overrideManEn)
{
	if (controlInputs.manual_control_enable || overrideManEn)
	{
		//update ESC speed
		throttle.write(constrain(controlInputs.throttle_command, THROTTLE_MIN, THROTTLE_MAX));

		//update servo positions
		elevator.writeMicroseconds(constrain(controlInputs.pitch_command, ELEVATOR_MIN, ELEVATOR_MAX));
		rudder.writeMicroseconds(constrain(controlInputs.yaw_command,     RUDDER_MIN,   RUDDER_MAX));
		aileron.writeMicroseconds(constrain(controlInputs.roll_command,   AILERON_MIN,  AILERON_MAX));
	}
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
		aileron.writeMicroseconds(constrain(value, AILERON_MIN, AILERON_MAX));
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
			if (limiterTimer.fire())
			{
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
	if (imuTimer.fire())
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
