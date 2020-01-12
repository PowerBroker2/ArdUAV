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
		myGS.telemetry.velocity         = ((GS_telemetryTransfer.rxBuff[0]  << 8) | GS_telemetryTransfer.rxBuff[1])  / 100.0;
		myGS.telemetry.altitude         = ((GS_telemetryTransfer.rxBuff[2]  << 8) | GS_telemetryTransfer.rxBuff[3])  / 100.0;
		myGS.telemetry.pitchAngle       = ((GS_telemetryTransfer.rxBuff[4]  << 8) | GS_telemetryTransfer.rxBuff[5])  / 100.0;
		myGS.telemetry.rollAngle        = ((GS_telemetryTransfer.rxBuff[6]  << 8) | GS_telemetryTransfer.rxBuff[7])  / 100.0;
		myGS.telemetry.latitude         = ((GS_telemetryTransfer.rxBuff[8]  << 8) | GS_telemetryTransfer.rxBuff[9])  / 100.0;
		myGS.telemetry.longitude        = ((GS_telemetryTransfer.rxBuff[10] << 8) | GS_telemetryTransfer.rxBuff[11]) / 100.0;
		myGS.telemetry.UTC_year         =  (GS_telemetryTransfer.rxBuff[12] << 8) | GS_telemetryTransfer.rxBuff[13];
		myGS.telemetry.UTC_month        =   GS_telemetryTransfer.rxBuff[14];
		myGS.telemetry.UTC_day          =   GS_telemetryTransfer.rxBuff[15];
		myGS.telemetry.UTC_hour         =   GS_telemetryTransfer.rxBuff[16];
		myGS.telemetry.UTC_minute       =   GS_telemetryTransfer.rxBuff[17];
		myGS.telemetry.UTC_second       = ((GS_telemetryTransfer.rxBuff[18] << 8) | GS_telemetryTransfer.rxBuff[19]) / 100.0;
		myGS.telemetry.speedOverGround  = ((GS_telemetryTransfer.rxBuff[20] << 8) | GS_telemetryTransfer.rxBuff[21]) / 100.0;
		myGS.telemetry.courseOverGround = ((GS_telemetryTransfer.rxBuff[22] << 8) | GS_telemetryTransfer.rxBuff[23]) / 100.0;
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
