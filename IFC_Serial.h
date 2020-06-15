#pragma once
#include "Arduino.h"

#include "Shared_Tools.h"




/////////////////////////////////////////////////////////////////////////////////////////
//macros
#define IFC_DEBUG_PORT_NUMBER     0 //USB
#define IFC_COMMAND_PORT_NUMBER   3 //Serial3
#define IFC_GPS_PORT_NUMBER       1 //Serial1
#define IFC_LIDAR_PORT_NUMBER     2 //Serial2
#define IFC_TELEM_PORT_NUMBER     5 //Serial5
/////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////////////
//DO NOT EDIT THIS BLOCK-----------------------------------------------------------------
#if IFC_DEBUG_PORT_NUMBER == 0
#define IFC_DEBUG_PORT Serial

#elif IFC_DEBUG_PORT_NUMBER == 1
#define IFC_DEBUG_PORT Serial1

#elif IFC_DEBUG_PORT_NUMBER == 2
#define IFC_DEBUG_PORT Serial2

#elif IFC_DEBUG_PORT_NUMBER == 3
#define IFC_DEBUG_PORT Serial3

#elif IFC_DEBUG_PORT_NUMBER == 4
#define IFC_DEBUG_PORT Serial4

#elif IFC_DEBUG_PORT_NUMBER == 5
#define IFC_DEBUG_PORT Serial5

#elif IFC_DEBUG_PORT_NUMBER == 6
#define IFC_DEBUG_PORT Serial6
#endif




#if IFC_COMMAND_PORT_NUMBER == 0
#define IFC_COMMAND_PORT Serial

#elif IFC_COMMAND_PORT_NUMBER == 1
#define IFC_COMMAND_PORT Serial1

#elif IFC_COMMAND_PORT_NUMBER == 2
#define IFC_COMMAND_PORT Serial2

#elif IFC_COMMAND_PORT_NUMBER == 3
#define IFC_COMMAND_PORT Serial3

#elif IFC_COMMAND_PORT_NUMBER == 4
#define IFC_COMMAND_PORT Serial4

#elif IFC_COMMAND_PORT_NUMBER == 5
#define IFC_COMMAND_PORT Serial5

#elif IFC_COMMAND_PORT_NUMBER == 6
#define IFC_COMMAND_PORT Serial6
#endif




#if IFC_LIDAR_PORT_NUMBER == 0
#define IFC_LIDAR_PORT Serial

#elif IFC_LIDAR_PORT_NUMBER == 1
#define IFC_LIDAR_PORT Serial1

#elif IFC_LIDAR_PORT_NUMBER == 2
#define IFC_LIDAR_PORT Serial2

#elif IFC_LIDAR_PORT_NUMBER == 3
#define IFC_LIDAR_PORT Serial3

#elif IFC_LIDAR_PORT_NUMBER == 4
#define IFC_LIDAR_PORT Serial4

#elif IFC_LIDAR_PORT_NUMBER == 5
#define IFC_LIDAR_PORT Serial5

#elif IFC_LIDAR_PORT_NUMBER == 6
#define IFC_LIDAR_PORT Serial6
#endif




#if IFC_GPS_PORT_NUMBER == 0
#define IFC_GPS_PORT Serial

#elif IFC_GPS_PORT_NUMBER == 1
#define IFC_GPS_PORT Serial1

#elif IFC_GPS_PORT_NUMBER == 2
#define IFC_GPS_PORT Serial2

#elif IFC_GPS_PORT_NUMBER == 3
#define IFC_GPS_PORT Serial3

#elif IFC_GPS_PORT_NUMBER == 4
#define IFC_GPS_PORT Serial4

#elif IFC_GPS_PORT_NUMBER == 5
#define IFC_GPS_PORT Serial5

#elif IFC_GPS_PORT_NUMBER == 6
#define IFC_GPS_PORT Serial6
#endif




#if IFC_TELEM_PORT_NUMBER == 0
#define IFC_TELEM_PORT Serial

#elif IFC_TELEM_PORT_NUMBER == 1
#define IFC_TELEM_PORT Serial1

#elif IFC_TELEM_PORT_NUMBER == 2
#define IFC_TELEM_PORT Serial2

#elif IFC_TELEM_PORT_NUMBER == 3
#define IFC_TELEM_PORT Serial3

#elif IFC_TELEM_PORT_NUMBER == 4
#define IFC_TELEM_PORT Serial4

#elif IFC_TELEM_PORT_NUMBER == 5
#define IFC_TELEM_PORT Serial5

#elif IFC_TELEM_PORT_NUMBER == 6
#define IFC_TELEM_PORT Serial6
#endif




#if IFC_GPS_PORT_NUMBER == IFC_DEBUG_PORT_NUMBER
#error "Can't have GPS and debug on same Serial port"
#endif
#if IFC_GPS_PORT_NUMBER == IFC_COMMAND_PORT_NUMBER
#error "Can't have GPS and command on same Serial port"
#endif
#if IFC_GPS_PORT_NUMBER == IFC_LIDAR_PORT_NUMBER
#error "Can't have GPS and LiDAR on same Serial port"
#endif
#if IFC_GPS_PORT_NUMBER == IFC_TELEM_PORT_NUMBER
#error "Can't have GPS and telemetry on same Serial port"
#endif




#if IFC_DEBUG_PORT_NUMBER == IFC_COMMAND_PORT_NUMBER
#error "Can't have debug and command on same Serial port"
#endif




#if IFC_DEBUG_PORT_NUMBER == IFC_TELEM_PORT_NUMBER && DEBUG_PORT_BAUD != TELEM_PORT_BAUD
#error "Debug and telem on same Serial port, but debug baud != telem baud"
#endif
#if IFC_TELEM_PORT_NUMBER == IFC_COMMAND_PORT_NUMBER && TELEM_PORT_BAUD != COMMAND_PORT_BAUD
#error "Telem and command on same Serial port, but telem baud != command baud"
#endif
//DO NOT EDIT THIS BLOCK-----------------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////