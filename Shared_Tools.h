#pragma once
#include "Arduino.h"




/////////////////////////////////////////////////////////////////////////////////////////
//macros
#define DEBUG_PORT_BAUD        115200
#define COMMAND_PORT_BAUD      115200
#define GPS_PORT_BAUD          9600
#define TELEM_PORT_BAUD        9600

#define REPORT_COMMANDS_FREQ   20.00 //Hz
#define REPORT_TELEM_FREQ      10.00 //Hz
#define REPORT_COMMANDS_PERIOD (byte)((1.0 / REPORT_COMMANDS_FREQ) * 1000) //ms
#define REPORT_TELEM_PERIOD    (byte)((1.0 / REPORT_TELEM_FREQ)    * 1000) //ms

#define LOSS_LINK_TIMEOUT      1000 //ms

#define ANALOG_RESOLUTION      16

#define ELEVATOR_INDEX         0
#define AILERON_INDEX          1
#define RUDDER_INDEX           2
#define THROTTLE_INDEX         3
#define NOSE_GEAR_INDEX        5

#define AILERON_OFFSET         0
#define ELEVATOR_OFFSET        0
#define RUDDER_OFFSET          0
#define THROTTLE_OFFSET        0

#define THROTTLE_MAX           170  //full throttle
#define AILERON_MAX            2000 //roll left
#define ELEVATOR_MAX           2000 //nose up
#define RUDDER_MAX             2000 //nose left

#define THROTTLE_MIN           50   //no throttle
#define AILERON_MIN            1000 //roll right
#define ELEVATOR_MIN           1000 //nose down
#define RUDDER_MIN             1000 //nose right

#define AILERON_MID            (uint16_t)((AILERON_MAX  + AILERON_MIN)  / 2)
#define ELEVATOR_MID           (uint16_t)((ELEVATOR_MAX + ELEVATOR_MIN) / 2)
#define RUDDER_MID             (uint16_t)((RUDDER_MAX   + RUDDER_MIN)   / 2)




/////////////////////////////////////////////////////////////////////////////////////////
//shared variables/flags
extern bool linkConnected;  //false if radio link between GS and IFC is lost
extern bool packetDetected; //true if packet is detected
/////////////////////////////////////////////////////////////////////////////////////////
