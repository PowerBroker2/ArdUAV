#include "GS_Serial.h"

#include "Shared_Tools.h"
#include "GS_Tools.h"
#include "SerialTransfer.h"




void commEvent_GS()
{
	if (GS_telemetryTransfer.available())
	{
		packetDetected = true;
		myGS.newTelem = true;

		//update telemetry struct with latest telemetry data
		GS_telemetryTransfer.rxObj(myGS.telemetry, sizeof(myGS.telemetry));
	}
	else if (GS_telemetryTransfer.status < 0)
	{
		packetDetected = false;

		GS_DEBUG_PORT.print("Telemetry Link Serial Transfer ERROR: ");
		GS_DEBUG_PORT.println(GS_telemetryTransfer.status);
	}
	else
		packetDetected = false;
}




/////////////////////////////////////////////////////////////////////////////////////////
//DO NOT EDIT THIS BLOCK-----------------------------------------------------------------
#if GS_TELEM_PORT_NUMBER == 0
void serialEvent()
{
	commEvent_GS();
}

#elif GS_TELEM_PORT_NUMBER == 1
void serialEvent1()
{
	commEvent_GS();
}

#elif GS_TELEM_PORT_NUMBER == 2
void serialEvent2()
{
	commEvent_GS();
}

#elif GS_TELEM_PORT_NUMBER == 3
void serialEvent3()
{
	commEvent_GS();
}

#elif GS_TELEM_PORT_NUMBER == 4
void serialEvent4()
{
	commEvent_GS();
}

#elif GS_TELEM_PORT_NUMBER == 5
void serialEvent5()
{
	commEvent_GS();
}

#elif GS_TELEM_PORT_NUMBER == 6
void serialEvent6()
{
	commEvent_GS();
}

#endif
//DO NOT EDIT THIS BLOCK-----------------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////
