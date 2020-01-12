#include "IFC_Tools.h"
#include "IFC_Serial.h"
#include "Shared_Tools.h"

#include "SerialTransfer.h"




void commEvent_IFC()
{
	if (IFC_commandTransfer.available())
	{
		packetDetected = true;

		//update controlInputs struct so that the next time the servos can be updated with the latest positions
		myIFC.controlInputs.pitch_command       = (IFC_commandTransfer.rxBuff[0] << 8)  | IFC_commandTransfer.rxBuff[1];
		myIFC.controlInputs.roll_command        = (IFC_commandTransfer.rxBuff[2] << 8)  | IFC_commandTransfer.rxBuff[3];
		myIFC.controlInputs.yaw_command         = (IFC_commandTransfer.rxBuff[4] << 8)  | IFC_commandTransfer.rxBuff[5];
		myIFC.controlInputs.throttle_command    = (IFC_commandTransfer.rxBuff[6] << 8)  | IFC_commandTransfer.rxBuff[7];
		myIFC.controlInputs.autopilot_command   = (IFC_commandTransfer.rxBuff[8] << 8)  | IFC_commandTransfer.rxBuff[9];
		myIFC.controlInputs.limiter_command     = (IFC_commandTransfer.rxBuff[10] << 8) | IFC_commandTransfer.rxBuff[11];
		myIFC.controlInputs.gear_command        = (IFC_commandTransfer.rxBuff[12] << 8) | IFC_commandTransfer.rxBuff[13];
		myIFC.controlInputs.flaps_command       = (IFC_commandTransfer.rxBuff[14] << 8) | IFC_commandTransfer.rxBuff[15];
		
		//tweak the contents of controlInputs to keep the plane from unsafe maneuvers
		//myIFC.bankPitchLimiter(myIFC.controlInputs.limiter_enable, true);
	}
	else if (IFC_commandTransfer.status < 0)
	{
		packetDetected = false;

		IFC_DEBUG_PORT.print("Command Link Serial Transfer ERROR: ");
		IFC_DEBUG_PORT.println(IFC_commandTransfer.status);

		//tweak the contents of controlInputs to keep the plane from unsafe maneuvers
		//update servos with new values automatically if there is a loss of link
		//myIFC.bankPitchLimiter(controlInputs.limiter_enable, false);
	}
	else
	{
		packetDetected = false;

		//tweak the contents of controlInputs to keep the plane from unsafe maneuvers
		//update servos with new values automatically if there is a loss of link
		//myIFC.bankPitchLimiter(controlInputs.limiter_enable, false);
	}
}




/////////////////////////////////////////////////////////////////////////////////////////
//DO NOT EDIT THIS BLOCK-----------------------------------------------------------------
#if IFC_COMMAND_PORT_NUMBER == 0
void serialEvent()
{
	commEvent_IFC();
}

#elif IFC_COMMAND_PORT_NUMBER == 1
void serialEvent1()
{
	commEvent_IFC();
}

#elif IFC_COMMAND_PORT_NUMBER == 2
void serialEvent2()
{
	commEvent_IFC();
}

#elif IFC_COMMAND_PORT_NUMBER == 3
void serialEvent3()
{
	commEvent_IFC();
}

#elif IFC_COMMAND_PORT_NUMBER == 4
void serialEvent4()
{
	commEvent_IFC();
}

#elif IFC_COMMAND_PORT_NUMBER == 5
void serialEvent5()
{
	commEvent_IFC();
}

#elif IFC_COMMAND_PORT_NUMBER == 6
void serialEvent6()
{
	commEvent_IFC();
}

#endif
//DO NOT EDIT THIS BLOCK-----------------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////
