#include "IFC_Tools.h"
#include "IFC_Serial.h"
#include "Shared_Tools.h"
#include "SerialTransfer.h"
#include "NMEAGPS.h"




#if USE_IFC_TELEM
void IFC_Class::sendTelem()
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




#if USE_IMU
void IFC_Class::pollIMU()
{
	//get IMU data and convert to degrees
	auto vect = bno.getQuat().toEuler();
	telemetry.courseAngleIMU = vect.x() * (180 / M_PI);
	telemetry.pitchAngle     = vect.y() * (180 / M_PI);
	telemetry.rollAngle      = vect.z() * (180 / M_PI);

	//convert Euler angles from degrees to radians - ONLY USED FOR LiDAR CORRECTION
	convertedPitch = vect.z();
	convertedRoll  = vect.y();

	//timestamp the new data - regardless of where this function was called
	imuTimer.start();
}




void IFC_Class::bankPitchLimiter()
{
	if (linkConnected)
	{
		updateControlsLimiter(PITCH_AXIS);
		updateControlsLimiter(ROLL_AXIS);
	}
	else
	{
		if (limiterTimer.fire())
		{
			updateControlsLimiter(PITCH_AXIS);
			updateControlsLimiter(ROLL_AXIS);

			updateServos();
		}
	}
}




void IFC_Class::updateControlsLimiter(const bool& axis)
{
	//minimum servo command to get back to safe flight
	uint16_t minServoCommand;

	//determine if the current IMU data is old - if so, get new IMU data
	if (imuTimer.fire())
		pollIMU();

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
#endif




#if USE_GPS
bool IFC_Class::pollGPS()
{
	if (gps.available(IFC_GPS_PORT))
	{
		fix = gps.read();
		readGPSData();

		lossGPSTimer.start();
		gpsConnected = true;
		telemetry.validFlags = telemetry.validFlags | 0x8;
	}

	return gpsFailover();
}




void IFC_Class::readGPSData()
{
	if (fix.valid.status)
	{
		if (fix.valid.location)
		{
			telemetry.latitude  = fix.latitude();
			telemetry.longitude = fix.longitude();

			//record that fix data is valid
			telemetry.validFlags = telemetry.validFlags | 0x6;
		}
		else
		{
			telemetry.latitude  = 0;
			telemetry.longitude = 0;

			//record that fix data is not valid
			telemetry.validFlags = telemetry.validFlags & ~(byte)0x6;
		}

		if (fix.valid.date)
		{
			telemetry.UTC_year  = fix.dateTime.year;
			telemetry.UTC_month = fix.dateTime.month;
			telemetry.UTC_day   = fix.dateTime.date;
		}
		else
		{
			telemetry.UTC_year  = 0;
			telemetry.UTC_month = 0;
			telemetry.UTC_day   = 0;
		}
		
		if (fix.valid.time)
		{
			telemetry.UTC_hour   = fix.dateTime.hours;
			telemetry.UTC_minute = fix.dateTime.minutes;
			telemetry.UTC_second = fix.dateTime.seconds;
		}
		else
		{
			telemetry.UTC_hour   = 0;
			telemetry.UTC_minute = 0;
			telemetry.UTC_second = 0;
		}

		if (fix.valid.speed)
			telemetry.speedOverGround = fix.speed_kph();
		else
			telemetry.speedOverGround = 0;

		if (fix.valid.heading)
			telemetry.courseOverGround = fix.heading();
		else
			telemetry.courseOverGround = 0;
	}
}




uint32_t IFC_Class::findBaud()
{
	for (uint8_t i = 0; i < (sizeof(bauds) / sizeof(bauds[0])); i++)
	{
		bool found = false;
		IFC_GPS_PORT.begin(bauds[i]);

		delay(100);

		while (IFC_GPS_PORT.available())
			IFC_GPS_PORT.read();

		autobaudTimer.begin(100);

		while (!autobaudTimer.fire())
		{
			if (IFC_GPS_PORT.available() >= 2)
			{
				char temp  = IFC_GPS_PORT.read();
				char temp2 = IFC_GPS_PORT.read();

				if (temp == '$' && temp2 == 'G')
				{
					found = true;
					break;
				}
			}
		}

		if (found)
			return bauds[i];
	}

	return 38400;
}




void IFC_Class::setupGPS()
{
	//disable all NMEA sentences first
	setSentence(GPGGA, false);
	setSentence(GPGLL, false);
	setSentence(GPGLV, false);
	setSentence(GPGSA, false);
	setSentence(GPRMC, false);
	setSentence(GPVTG, false);

	//then select the ones you want
	if (GPGGA_ENABLED)
		setSentence(GPGGA, true);
	if (GPGLL_ENABLED)
		setSentence(GPGLL, true);
	if (GPGLV_ENABLED)
		setSentence(GPGLV, true);
	if (GPGSA_ENABLED)
		setSentence(GPGSA, true);
	if (GPRMC_ENABLED)
		setSentence(GPRMC, true);
	if (GPVTG_ENABLED)
		setSentence(GPVTG, true);

	changeFreq(GPS_REFRESH);

	if (GPS_PORT_BAUD != foundBaud)
		changeBaud(GPS_PORT_BAUD);
}




void changeBaud(uint32_t baud)
{
	char configPacket[BAUD_LEN];
	memcpy(configPacket, CFG_PRT, BAUD_LEN);

	configPacket[BAUD_0] = (char)(baud & 0xFF);
	configPacket[BAUD_1] = (char)((baud >> 8) & 0xFF);
	configPacket[BAUD_2] = (char)((baud >> 16) & 0xFF);
	configPacket[BAUD_3] = (char)((baud >> 24) & 0xFF);

	insertChecksum(configPacket,     BAUD_LEN);
	IFC_GPS_PORT.write(configPacket, BAUD_LEN);

	delay(100);

	IFC_GPS_PORT.flush();
	IFC_GPS_PORT.begin(baud);
}




void setSentence(char NMEA_num, bool enable)
{
	char configPacket[NMEA_LEN];
	memcpy(configPacket, CFG_MSG, NMEA_LEN);

	if (enable)
		configPacket[SERIAL_1_POS] = 1;

	configPacket[NMEA_ID_POS] = NMEA_num;
	insertChecksum(configPacket, NMEA_LEN);

	IFC_GPS_PORT.write(configPacket, NMEA_LEN);
}




void changeFreq(uint16_t hertz)
{
	uint16_t normHerz = hertz / (1000 / ((CFG_RATE[MEAS_RATE_2] << 8) | CFG_RATE[MEAS_RATE_1]));
	char configPacket[FREQ_LEN];
	memcpy(configPacket, CFG_RATE, FREQ_LEN);

	configPacket[NAV_RATE_1] = (char)(normHerz & 0xFF);
	configPacket[NAV_RATE_2] = (char)((normHerz >> 8) & 0xFF);

	insertChecksum(configPacket, FREQ_LEN);
	IFC_GPS_PORT.write(configPacket, FREQ_LEN);
}




void insertChecksum(char packet[], const byte len)
{
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;

	// exclude the first and last two bytes in packet
	for (byte i = 2; i < (len - 2); i++)
	{
		ck_a += packet[i];
		ck_b += ck_a;
	}

	packet[len - 2] = ck_a;
	packet[len - 1] = ck_b;
}




bool IFC_Class::gpsFailover()
{
	if (lossGPSTimer.fire(false))
	{
		gpsConnected = false;
		telemetry.validFlags = telemetry.validFlags & ~(byte)0x8;
	}

	return gpsConnected;
}
#endif




#if USE_PITOT
void IFC_Class::pollPitot()
{
	telemetry.pitotPressure = analogRead(PITOT_PIN);
}
#endif




#if USE_LIDAR
void IFC_Class::lidarEvent()
{
	if (lidarTransfer.available())
	{
		//update controlInputs struct so that the next time the servos can be updated with the latest positions
		lidarTransfer.rxObj(telemetry.altitude);

		//use trig to find the triangulated elevation if the LiDAR sensor is not stabilized with a gimbal
		if (LIDAR_FIXED_MOUNT)
#if USE_IMU
			telemetry.altitude = telemetry.altitude * cos(abs(convertedRoll)) * cos(abs(convertedPitch));
#else
			telemetry.altitude = telemetry.altitude;
#endif
		else
			telemetry.altitude = telemetry.altitude;
	}
}
#endif




void IFC_Class::begin()
{
	//initialize variables
	controlInputs.limiter_enable = false;
	controlInputs.manual_control_enable = true;




	//wait for all serial ports to come online
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("Initializing serial ports..."));
#endif

#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.print(F("Initializing command port at Serial")); IFC_DEBUG_PORT.print(IFC_COMMAND_PORT_NUMBER);  IFC_DEBUG_PORT.println(F("..."));
#endif
	IFC_COMMAND_PORT.begin(COMMAND_PORT_BAUD);

#if USE_GPS
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.print(F("Initializing GPS port at Serial")); IFC_DEBUG_PORT.print(IFC_GPS_PORT_NUMBER);  IFC_DEBUG_PORT.println(F("..."));
#endif
	foundBaud = findBaud();
#endif

#if USE_LIDAR
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.print(F("Initializing LiDAR port at Serial")); IFC_DEBUG_PORT.print(IFC_LIDAR_PORT_NUMBER);  IFC_DEBUG_PORT.println(F("..."));
#endif
	IFC_LIDAR_PORT.begin(LIDAR_PORT_BAUD);
#endif

#if USE_IFC_TELEM
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.print(F("Initializing telemetry at Serial")); IFC_DEBUG_PORT.print(IFC_TELEM_PORT_NUMBER);  IFC_DEBUG_PORT.println(F("..."));
#endif
	IFC_TELEM_PORT.begin(TELEM_PORT_BAUD);
#endif

#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("\tAll ports sucessfully initialized"));
#endif




	//turn on PWR LED
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("Turning on PWR LED..."));
#endif

	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("\tPWR LED on"));
#endif




	//set analog to digital conversion resolution (in bits)
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("Setting ADC resolution (default - 16 bits)..."));
#endif

	analogReadResolution(ANALOG_RESOLUTION);

#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("\tResolution set"));
#endif




	//initialize transfer classes
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("Initializing transfer classes..."));
#endif

	commandTransfer.begin(IFC_COMMAND_PORT);

#if USE_IFC_TELEM
	telemetryTransfer.begin(IFC_TELEM_PORT);
	telemetry.validFlags = 0;
#endif

#if USE_LIDAR
	lidarTransfer.begin(IFC_LIDAR_PORT);
#endif

#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("\tTransfer classes initialized"));
#endif




	//initialize GPS
#if USE_GPS
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("Initializing GPS..."));
#endif

	setupGPS();

#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("\tGPS initialized"));
#endif
#endif




	//initialize the IMU and wait for it to "boot up"
#if USE_IMU
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("Initializing IMU..."));
#endif

	if (!bno.begin())
	{
		//there was a problem detecting the BNO055 ... check your connections
#if USE_IFC_DEBUG
		IFC_DEBUG_PORT.println(F("\tOoops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
#endif

		//fatal error - halt code execution
		while (1);
	}
	else
	{
		delay(1000);
		bno.setExtCrystalUse(true);

#if USE_IFC_DEBUG
		IFC_DEBUG_PORT.println(F("\tBNO055 successfully initialized"));
#endif
	}
#endif




	//initialize each individual servo to their respective center position
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("Initializing servos..."));
#endif

	rudder.attach(RUDDER_SERVO_PIN);
	elevator.attach(ELEVATOR_SERVO_PIN);
	aileron.attach(AILERON_SERVO_PIN);

	rudder.writeMicroseconds(RUDDER_MID);
	elevator.writeMicroseconds(ELEVATOR_MID);
	aileron.writeMicroseconds(AILERON_MID);

#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("\tServos initialized..."));
#endif




	//initialize the ESC
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("Initializing ESC..."));
#endif

	throttle.attach(THROTTLE_SERVO_PIN);
	throttle.write(THROTTLE_MIN);

#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("\tESC initialized..."));
#endif




	//initialize the timers
#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("Initializing timers..."));
#endif

#if USE_IMU
	limiterTimer.begin(LIMITER_PERIOD);
#endif

	lossLinkTimer.begin(LOSS_LINK_TIMEOUT);
	telemTimer.begin(REPORT_TELEM_PERIOD);

#if USE_GPS
	lossGPSTimer.begin(LOSS_GPS_TIMEOUT);
#endif

#if USE_IMU
	imuTimer.begin(LIMITER_PERIOD);
#endif

#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("\tTimers initialized..."));
#endif




#if USE_IFC_DEBUG
	IFC_DEBUG_PORT.println(F("Initialization complete"));
	IFC_DEBUG_PORT.println(F("--------------------------------------------------"));
#endif
}




bool IFC_Class::tick()
{
#if USE_IFC_TELEM
	sendTelem();
#endif

#if USE_IMU
	pollIMU();
#endif

#if USE_GPS
	pollGPS();
#endif

#if USE_PITOT
	pollPitot();
#endif

#if USE_LIDAR
	lidarEvent();
#endif

	return commEvent();
}




bool IFC_Class::commEvent()
{
	if (commandTransfer.available())
	{
		commandTransfer.rxObj(controlInputs);

		lossLinkTimer.start();
		linkConnected = true;
		telemetry.validFlags = telemetry.validFlags | 0x2;
	}

	return linkFailover();
}




//update servo positions (use controlInputs commands)
void IFC_Class::updateServos(const bool& overrideManEn)
{
	if (controlInputs.manual_control_enable || overrideManEn)
	{
		throttle.write(constrain(controlInputs.throttle_command, THROTTLE_MIN, THROTTLE_MAX));

		elevator.writeMicroseconds(constrain(controlInputs.pitch_command, ELEVATOR_MIN, ELEVATOR_MAX));
		rudder.writeMicroseconds(constrain(controlInputs.yaw_command,     RUDDER_MIN,   RUDDER_MAX));
		aileron.writeMicroseconds(constrain(controlInputs.roll_command,   AILERON_MIN,  AILERON_MAX));
	}
}




//update a single servo's position (use controlInputs commands)
void IFC_Class::updateSingleServo(const byte& INDEX, const uint16_t& value)
{
	if (INDEX == COMMAND_THROTTLE_INDEX)
		throttle.write(constrain(value, THROTTLE_MIN, THROTTLE_MAX));
	
	else if (INDEX == COMMAND_PITCH_INDEX)
		elevator.writeMicroseconds(constrain(value, ELEVATOR_MIN, ELEVATOR_MAX));
	
	else if (INDEX == COMMAND_YAW_INDEX)
		rudder.writeMicroseconds(constrain(value, RUDDER_MIN, RUDDER_MAX));
	
	else if (INDEX == COMMAND_ROLL_INDEX)
	{
		aileron.writeMicroseconds(constrain(value, AILERON_MIN, AILERON_MAX));
	}
	else
		IFC_DEBUG_PORT.println(F("Servo Not recognized"));
}




bool IFC_Class::linkFailover()
{
	if (lossLinkTimer.fire(false))
	{
		controlInputs.pitch_command    = 1500;
		controlInputs.roll_command     = 1500;
		controlInputs.yaw_command      = 1500;
		controlInputs.throttle_command = 1500;

		updateServos();

		linkConnected = false;
		telemetry.validFlags = telemetry.validFlags & ~(byte)0x2;
	}

	return linkConnected;
}
