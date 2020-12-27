#pragma once
#include "Arduino.h"
#include "Wire.h"
#include "SerialTransfer.h"
#include "FireTimer.h"




extern TwoWire Wire1;
extern TwoWire Wire2;




//macros
#define USE_GS_DEBUG  1
#define USE_IFC_DEBUG 1
#define USE_GS_TELEM  1
#define USE_IFC_TELEM 1
#define USE_IMU       1
#define USE_GPS       1
#define USE_PITOT     1
#define USE_LIDAR     1

#define IMU_PORT Wire
#define IMU_ID   55

#define DEBUG_PORT_BAUD       115200
#define COMMAND_PORT_BAUD     115200
#define GPS_PORT_BAUD         115200
#define LIDAR_PORT_BAUD       115200
#define TELEM_PORT_BAUD       115200

#define IFC_DEBUG_PORT_NUMBER   0 //USB
#define IFC_COMMAND_PORT_NUMBER 3 //Serial3
#define IFC_GPS_PORT_NUMBER     4 //Serial4
#define IFC_LIDAR_PORT_NUMBER   2 //Serial2
#define IFC_TELEM_PORT_NUMBER   1 //Serial1

#define GS_DEBUG_PORT_NUMBER   0 //USB
#define GS_COMMAND_PORT_NUMBER 1 //Serial1
#define GS_TELEM_PORT_NUMBER   2 //Serial2

#define COMMAND_BUFFER   10
#define TELEMETRY_BUFFER 10

#define REPORT_COMMANDS_FREQ   50.00 //Hz
#define REPORT_TELEM_FREQ      20.00 //Hz
#define REPORT_COMMANDS_PERIOD (byte)((1.0 / REPORT_COMMANDS_FREQ) * 1000) //ms
#define REPORT_TELEM_PERIOD    (byte)((1.0 / REPORT_TELEM_FREQ)    * 1000) //ms

#define LOSS_LINK_TIMEOUT 1000 //ms
#define LOSS_GPS_TIMEOUT  1000 //ms

#define ANALOG_RESOLUTION 16

#define COMMAND_ROLL_INDEX     0
#define COMMAND_PITCH_INDEX    1
#define COMMAND_YAW_INDEX      2
#define COMMAND_THROTTLE_INDEX 3

#define AILERON_OFFSET  0
#define ELEVATOR_OFFSET 0
#define RUDDER_OFFSET   0
#define THROTTLE_OFFSET 0

#define AILERON_SERVO_PIN  4 //servo driver output port number
#define ELEVATOR_SERVO_PIN 5 //servo driver output port number
#define RUDDER_SERVO_PIN   6 //servo driver output port number
#define THROTTLE_SERVO_PIN 3 //digital ESC signal pin

#define ROLL_ANALOG_PIN     A8
#define PITCH_ANALOG_PIN    A9
#define YAW_ANALOG_PIN      A16
#define THROTTLE_ANALOG_PIN A17

#define AILERON_MAX_ADC  41800
#define ELEVATOR_MAX_ADC 40900
#define RUDDER_MAX_ADC   41600
#define THROTTLE_MAX_ADC 40850

#define AILERON_MIN_ADC  25000
#define ELEVATOR_MIN_ADC 23600
#define RUDDER_MIN_ADC   24800
#define THROTTLE_MIN_ADC 23900

#define AILERON_REVERSE  1
#define ELEVATOR_REVERSE 1
#define RUDDER_REVERSE   1
#define THROTTLE_REVERSE 0

#define AILERON_MAX  2000 //roll left
#define ELEVATOR_MAX 2000 //nose up
#define RUDDER_MAX   2000 //nose left
#define THROTTLE_MAX 170  //full throttle

#define AILERON_MIN  1000 //roll right
#define ELEVATOR_MIN 1000 //nose down
#define RUDDER_MIN   1000 //nose right
#define THROTTLE_MIN 50   //no throttle

#define AILERON_MID  (uint16_t)((AILERON_MAX  + AILERON_MIN)  / 2)
#define ELEVATOR_MID (uint16_t)((ELEVATOR_MAX + ELEVATOR_MIN) / 2)
#define RUDDER_MID   (uint16_t)((RUDDER_MAX   + RUDDER_MIN)   / 2)

#define LIDAR_FIXED_MOUNT 0  //0 - gimbal mount, 0 - fixed mount

#define GPS_REFRESH   10
#define GPGGA_ENABLED 0
#define GPGLL_ENABLED 0
#define GPGLV_ENABLED 0
#define GPGSA_ENABLED 0
#define GPRMC_ENABLED 1
#define GPVTG_ENABLED 0

#define SERVO_FREQ     60 //Hz
#define LIMITER_PERIOD REPORT_COMMANDS_PERIOD //ms

#define PITOT_PIN A17 //analog input pin

#define PITCH_AXIS true  //
#define ROLL_AXIS  false //

#define UNSAFE_ROLL_R -35 //unsafe right bank angle threshold (in degrees)
#define UNSAFE_ROLL_L 35  //unsafe left bank angle threshold (in degrees)

#define MAX_ROLL_R -50 //max right bank angle (in degrees) allowed by flight controller
#define MAX_ROLL_L 50  //max left bank angle (in degrees) allowed by flight controller

#define UNSAFE_PITCH_UP   -30 //unsafe up pitch angle threshold (in degrees)
#define UNSAFE_PITCH_DOWN 10  //unsafe down pitch angle threshold (in degrees)

#define MAX_PITCH_UP   -45 //max up pitch angle (in degrees) allowed by flight controller
#define MAX_PITCH_DOWN 25  //max down pitch angle (in degrees) allowed by flight controller




float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);




//struct to handle control surface commands
struct __attribute__((__packed__)) control_surfaces_struct
{
	uint8_t command_index;
	uint8_t analog_pin;
	uint16_t _offset;
	uint16_t surface_max;
	uint16_t surface_min;
	uint16_t ADC_max;
	uint16_t ADC_min;
	bool reverse;
};

//struct to store telemetry data
struct __attribute__((__packed__)) telemetry_struct
{
	uint16_t altitude;      //cm
	float courseAngleIMU;   //degrees
	float rollAngle;        //degrees
	float pitchAngle;       //degrees
	uint16_t pitotPressure; //unitless             MSB                                                                        LSB
	uint8_t validFlags;     //bit encoded bools (reserved, reserved, imuRecent, imuValid, fixRecent, fixValid, linkValid, manualControl)
	float latitude;         //dd
	float longitude;        //dd
	uint16_t UTC_year;      //y
	uint8_t UTC_month;      //M
	uint8_t UTC_day;        //d
	uint8_t UTC_hour;       //h
	uint8_t UTC_minute;     //m
	float UTC_second;       //s
	float speedOverGround;  //knots
	float courseOverGround; //degrees
};

//struct to store control values (i.e. servo commands etc.)
struct __attribute__((__packed__)) control_inputs_struct
{
	bool limiter_enable;        //enables and disables pitch and bank limiter
	bool manual_control_enable; //enables and disables manual flight control
	uint16_t pitch_command;
	uint16_t roll_command;
	uint16_t yaw_command;
	uint16_t throttle_command;
	uint16_t autopilot_command;
	uint16_t limiter_command;
	uint16_t gear_command;
	uint16_t flaps_command;
};




//base class
class base
{
public:
	bool linkConnected;
	SerialTransfer commandTransfer;
	FireTimer commandTimer;
	FireTimer telemTimer;
	FireTimer lossLinkTimer;
	telemetry_struct telemetry;
	control_inputs_struct controlInputs;

	virtual void begin() = 0;
	virtual bool tick()  = 0;

protected:
	virtual bool linkFailover() = 0;
};
