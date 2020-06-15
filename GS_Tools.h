#pragma once
#include "Arduino.h"
#include "FireTimer.h"

#include "GS_Serial.h"
#include "Shared_Tools.h"
#include "SerialTransfer.h"




/////////////////////////////////////////////////////////////////////////////////////////
//macros
#define GS

#define YAW_ANALOG_PIN          A17
#define THROTTLE_ANALOG_PIN     A0
#define ROLL_ANALOG_PIN         A2
#define PITCH_ANALOG_PIN        A3

#define PITCH_RATE_PIN          0
#define ROLL_RATE_PIN           1
#define YAW_RATE_PIN            2

#define AILERON_MAX_LOWRATES    2000
#define ELEVATOR_MAX_LOWRATES   2000
#define RUDDER_MAX_LOWRATES     2000

#define AILERON_MIN_LOWRATES    1000
#define ELEVATOR_MIN_LOWRATES   1000
#define RUDDER_MIN_LOWRATES     1000

#define THROTTLE_MIN_ADC        23900
#define AILERON_MIN_ADC         25000
#define ELEVATOR_MIN_ADC        23600
#define RUDDER_MIN_ADC          24800

#define THROTTLE_MAX_ADC        40850
#define AILERON_MAX_ADC         41800
#define ELEVATOR_MAX_ADC        40900
#define RUDDER_MAX_ADC          41600

#define AILERON_REVERSE         1
#define ELEVATOR_REVERSE        1
#define RUDDER_REVERSE          1
#define THROTTLE_REVERSE        0

/////////////////////////////////////////////////////////////////////////////////////////
//DO NOT EDIT THIS BLOCK-----------------------------------------------------------------
#define AIR_ROLL_INDEX         0
#define AIR_PITCH_INDEX        1
#define AIR_YAW_INDEX          2
#define AIR_THROTTLE_INDEX     3
/////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////////////
//GS Class
class GS_Class
{
public:
	bool newTelem = false;

	//struct to store telemetry data
	telemetry_struct telemetry;

	//struct to store control values (i.e. servo commands etc.)
	controlInputs_struct controlInputs;




	void begin();
	void sendTelem();
	void computeCommands();
	void sendCommands();
	void computeAndSendCommands();




private:
	FireTimer commandTimer;
	



	/////////////////////////////////////////////////////////////////////////////////////////
	//structs to handle control surface commands
	struct controlSurfaces
	{
		byte command_index;
		byte analog_pin;
		byte rate_pin;
		uint16_t _offset;
		uint16_t high_rates_surface_max;
		uint16_t high_rates_surface_min;
		uint16_t low_rates_surface_max;
		uint16_t low_rates_surface_min;
		uint16_t ADC_max;
		uint16_t ADC_min;
		bool reverse;
	};

	struct controlSurfaces ailerons
	{
		AIR_ROLL_INDEX,         //command_index
		ROLL_ANALOG_PIN,        //analog_pin
		ROLL_RATE_PIN,          //rate_pin
		AILERON_OFFSET,         //_offset
		AILERON_MAX,            //high_rates_surface_max
		AILERON_MIN,            //high_rates_surface_min
		AILERON_MAX_LOWRATES,   //low_rates_surface_max
		AILERON_MIN_LOWRATES,   //low_rates_surface_min
		AILERON_MAX_ADC,        //ADC_max
		AILERON_MIN_ADC,        //ADC_min
		AILERON_REVERSE,        //flag to reverse servo throw direction
	};

	struct controlSurfaces elevator
	{
		AIR_PITCH_INDEX,        //command_index
		PITCH_ANALOG_PIN,       //analog_pin
		PITCH_RATE_PIN,         //rate_pin
		ELEVATOR_OFFSET,        //_offset
		ELEVATOR_MAX,           //high_rates_surface_max
		ELEVATOR_MIN,           //high_rates_surface_min
		ELEVATOR_MAX_LOWRATES,  //low_rates_surface_max
		ELEVATOR_MIN_LOWRATES,  //low_rates_surface_min
		ELEVATOR_MAX_ADC,       //ADC_max
		ELEVATOR_MIN_ADC,       //ADC_min
		ELEVATOR_REVERSE,       //flag to reverse servo throw direction
	};

	struct controlSurfaces rudder
	{
		AIR_YAW_INDEX,          //command_index
		YAW_ANALOG_PIN,         //analog_pin
		YAW_RATE_PIN,           //rate_pin
		RUDDER_OFFSET,          //_offset
		RUDDER_MAX,             //high_rates_surface_max
		RUDDER_MIN,             //high_rates_surface_min
		RUDDER_MAX_LOWRATES,    //low_rates_surface_max
		RUDDER_MIN_LOWRATES,    //low_rates_surface_min
		RUDDER_MAX_ADC,         //ADC_max
		RUDDER_MIN_ADC,         //ADC_max
		RUDDER_REVERSE,         //flag to reverse servo throw direction
	};

	struct controlSurfaces throttle
	{
		AIR_THROTTLE_INDEX,     //command_index
		THROTTLE_ANALOG_PIN,    //analog_pin
		0,                      //rate_pin
		THROTTLE_OFFSET,        //_offset
		THROTTLE_MAX,           //high_rates_surface_max
		THROTTLE_MIN,           //high_rates_surface_min
		0,                      //low_rates_surface_max
		0,                      //low_rates_surface_min
		THROTTLE_MAX_ADC,       //ADC_max
		THROTTLE_MIN_ADC,       //ADC_max
		THROTTLE_REVERSE,       //flag to reverse motor direction
	};
	/////////////////////////////////////////////////////////////////////////////////////////




	int16_t updateServoCommand(controlSurfaces controlSurface);
	bool checkRadioLink();
};




extern GS_Class myGS;
extern SerialTransfer GS_commandTransfer;
extern SerialTransfer GS_telemetryTransfer;
/////////////////////////////////////////////////////////////////////////////////////////
