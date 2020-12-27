#pragma once
#include "Arduino.h"
#include "Servo.h"
#include "NMEAGPS.h"
#include "FireTimer.h"
#include "IFC_Serial.h"
#include "Shared_Tools.h"
#include "SerialTransfer.h"
#include "ArdUAV_Adafruit_BNO055.h"




#if USE_GPS
#define NMEA_LEN 16
#define FREQ_LEN 14
#define BAUD_LEN 28

#define GPGGA 0
#define GPGLL 1
#define GPGLV 2
#define GPGSA 3
#define GPRMC 4
#define GPVTG 5

#define NMEA_ID_POS  7
#define DDC_POS      8
#define SERIAL_1_POS 9
#define SERIAL_2_POS 10
#define USB_POS      11
#define SPI_POS      12

#define BAUD_0 14
#define BAUD_1 15
#define BAUD_2 16
#define BAUD_3 17

#define MEAS_RATE_1 6
#define MEAS_RATE_2 7
#define NAV_RATE_1  8
#define NAV_RATE_2  9




const char CFG_MSG[NMEA_LEN] = {
	0xB5, // Header char 1
	0x62, // Header char 2
	0x06, // class
	0x01, // id
	0x08, // length LSB
	0x00, // length MSB
	0xF0, // payload (NMEA sentence ID char 1)
	0x00, // payload (NMEA sentence ID char 2)
	0x00, // payload I/O Target 0 - DDC           - (1 - enable sentence, 0 - disable)
	0x00, // payload I/O Target 1 - Serial Port 1 - (1 - enable sentence, 0 - disable)
	0x00, // payload I/O Target 2 - Serial Port 2 - (1 - enable sentence, 0 - disable)
	0x00, // payload I/O Target 3 - USB           - (1 - enable sentence, 0 - disable)
	0x00, // payload I/O Target 4 - SPI           - (1 - enable sentence, 0 - disable)
	0x00, // payload I/O Target 5 - Reserved      - (1 - enable sentence, 0 - disable)
	0x00, // CK_A
	0x00  // CK_B
};

const char CFG_RATE[FREQ_LEN] = {
	0xB5, // sync char 1
	0x62, // sync char 2
	0x06, // class
	0x08, // id
	0x06, // length LSB
	0x00, // length MSB
	0x64, // payload measRate (ms) 1
	0x00, // payload measRate (ms) 2
	0x00, // payload navRate (cycles) 1
	0x00, // payload navRate (cycles) 2
	0x01, // payload timeRef 1
	0x00, // payload timeRef 2
	0x00, // CK_A
	0x00  // CK_B
};

const char CFG_PRT[BAUD_LEN] = {
	0xB5, // sync char 1
	0x62, // sync char 2
	0x06, // class
	0x00, // id
	0x14, // length LSB
	0x00, // length MSB
	0x01, // payload portID
	0x00, // payload reserved0
	0x00, // payload txReady 1
	0x00, // payload txReady 2
	0xD0, // payload mode 1
	0x08, // payload mode 2
	0x00, // payload mode 3
	0x00, // payload mode 4
	0x00, // payload baudRate 0 (LSB)
	0x00, // payload baudRate 1
	0x00, // payload baudRate 2
	0x00, // payload baudRate 3 (MSB)
	0x07, // payload inProtoMask 1
	0x00, // payload inProtoMask 2
	0x03, // payload outProtoMask 1
	0x00, // payload outProtoMask 2
	0x00, // payload reserved4 1
	0x00, // payload reserved4 2
	0x00, // payload reserved5 1
	0x00, // payload reserved5 2
	0x00, // CK_A
	0x00  // CK_B
};




void changeBaud(uint32_t baud);
void changeFreq(uint16_t hertz);
void setSentence(char NMEA_num, bool enable);
void insertChecksum(char packet[], const byte len);
#endif




class IFC_Class : public base
{
public:
#if USE_IFC_TELEM
	SerialTransfer telemetryTransfer;

	void sendTelem();
#endif

#if USE_IMU
	Adafruit_BNO055 bno = Adafruit_BNO055(&IMU_PORT, IMU_ID, BNO055_ADDRESS_A);
	void pollIMU();
	void bankPitchLimiter();
#endif

#if USE_GPS
	bool gpsConnected;
	NMEAGPS gps;
	gps_fix fix;
	bool pollGPS();
#endif

#if USE_PITOT
	void pollPitot();
#endif

#if USE_LIDAR
	SerialTransfer lidarTransfer;
#endif

	Servo rudder;
	Servo elevator;
	Servo aileron;
	Servo throttle;

	void begin();
	bool tick();
	bool commEvent();
	void updateServos(const bool& overrideManEn=false);
	void updateSingleServo(const byte& INDEX, const uint16_t& value);




private:
#if USE_IMU
	float convertedPitch;
	float convertedRoll;
	unsigned long dataTimestamp_IMU;
	FireTimer limiterTimer;
	FireTimer imuTimer;
	void updateControlsLimiter(const bool& axis);
#endif

#if USE_GPS
	FireTimer lossGPSTimer;
	FireTimer autobaudTimer;

	uint32_t bauds[12]   = { 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200 };
	uint32_t foundBaud = 38400;

	uint32_t findBaud();
	void setupGPS();
	void readGPSData();
	bool gpsFailover();
#endif

#if USE_LIDAR
	byte LiDAR_Counter;
	void lidarEvent();
#endif

	bool linkFailover();
};
