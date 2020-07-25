#pragma once

#include <stdarg.h>
#include <inttypes.h>
#include <stddef.h>

#include "driver/uart.h"
#include "driver/gpio.h"

class RoboClaw
{
	uint16_t crc;
	uint32_t timeout = 10000; // [us]
	uart_port_t uart_num;
	gpio_num_t rxPin;
	gpio_num_t txPin;
	long speed;
	uint8_t address;

	enum
	{
		M1FORWARD = 0,
		M1BACKWARD = 1,
		SETMINMB = 2,
		SETMAXMB = 3,
		M2FORWARD = 4,
		M2BACKWARD = 5,
		M17BIT = 6,
		M27BIT = 7,
		MIXEDFORWARD = 8,
		MIXEDBACKWARD = 9,
		MIXEDRIGHT = 10,
		MIXEDLEFT = 11,
		MIXEDFB = 12,
		MIXEDLR = 13,
		GETM1ENC = 16,
		GETM2ENC = 17,
		GETM1SPEED = 18,
		GETM2SPEED = 19,
		RESETENC = 20,
		GETVERSION = 21,
		SETM1ENCCOUNT = 22,
		SETM2ENCCOUNT = 23,
		GETMBATT = 24,
		GETLBATT = 25,
		SETMINLB = 26,
		SETMAXLB = 27,
		SETM1PID = 28,
		SETM2PID = 29,
		GETM1ISPEED = 30,
		GETM2ISPEED = 31,
		M1DUTY = 32,
		M2DUTY = 33,
		MIXEDDUTY = 34,
		M1SPEED = 35,
		M2SPEED = 36,
		MIXEDSPEED = 37,
		M1SPEEDACCEL = 38,
		M2SPEEDACCEL = 39,
		MIXEDSPEEDACCEL = 40,
		M1SPEEDDIST = 41,
		M2SPEEDDIST = 42,
		MIXEDSPEEDDIST = 43,
		M1SPEEDACCELDIST = 44,
		M2SPEEDACCELDIST = 45,
		MIXEDSPEEDACCELDIST = 46,
		GETBUFFERS = 47,
		GETPWMS = 48,
		GETCURRENTS = 49,
		MIXEDSPEED2ACCEL = 50,
		MIXEDSPEED2ACCELDIST = 51,
		M1DUTYACCEL = 52,
		M2DUTYACCEL = 53,
		MIXEDDUTYACCEL = 54,
		READM1PID = 55,
		READM2PID = 56,
		SETMAINVOLTAGES = 57,
		SETLOGICVOLTAGES = 58,
		GETMINMAXMAINVOLTAGES = 59,
		GETMINMAXLOGICVOLTAGES = 60,
		SETM1POSPID = 61,
		SETM2POSPID = 62,
		READM1POSPID = 63,
		READM2POSPID = 64,
		M1SPEEDACCELDECCELPOS = 65,
		M2SPEEDACCELDECCELPOS = 66,
		MIXEDSPEEDACCELDECCELPOS = 67,
		SETM1DEFAULTACCEL = 68,
		SETM2DEFAULTACCEL = 69,
		SETPINFUNCTIONS = 74,
		GETPINFUNCTIONS = 75,
		SETDEADBAND = 76,
		GETDEADBAND = 77,
		GETENCODERS = 78,
		GETISPEEDS = 79,
		RESTOREDEFAULTS = 80,
		GETTEMP = 82,
		GETTEMP2 = 83, //Only valid on some models
		GETERROR = 90,
		GETENCODERMODE = 91,
		SETM1ENCODERMODE = 92,
		SETM2ENCODERMODE = 93,
		WRITENVM = 94,
		READNVM = 95, //Reloads values from Flash into Ram
		SETCONFIG = 98,
		GETCONFIG = 99,
		SETM1MAXCURRENT = 133,
		SETM2MAXCURRENT = 134,
		GETM1MAXCURRENT = 135,
		GETM2MAXCURRENT = 136,
		SETPWMMODE = 148,
		GETPWMMODE = 149,
		FLAGBOOTLOADER = 255 //Only available via USB communications
	};

public:
	RoboClaw(uart_port_t uart_num, gpio_num_t rxPin, gpio_num_t txPin, long speed, uint8_t address);

	bool ForwardM1(uint8_t speed);
	bool BackwardM1(uint8_t speed);
	bool SetMinVoltageMainBattery(uint8_t voltage);
	bool SetMaxVoltageMainBattery(uint8_t voltage);
	bool ForwardM2(uint8_t speed);
	bool BackwardM2(uint8_t speed);
	bool ForwardBackwardM1(uint8_t speed);
	bool ForwardBackwardM2(uint8_t speed);
	bool ForwardMixed(uint8_t speed);
	bool BackwardMixed(uint8_t speed);
	bool TurnRightMixed(uint8_t speed);
	bool TurnLeftMixed(uint8_t speed);
	bool ForwardBackwardMixed(uint8_t speed);
	bool LeftRightMixed(uint8_t speed);
	uint32_t ReadEncM1(uint8_t *status = NULL, bool *valid = NULL);
	uint32_t ReadEncM2(uint8_t *status = NULL, bool *valid = NULL);
	bool SetEncM1(int32_t val);
	bool SetEncM2(int32_t val);
	uint32_t ReadSpeedM1(uint8_t *status = NULL, bool *valid = NULL);
	uint32_t ReadSpeedM2(uint8_t *status = NULL, bool *valid = NULL);
	bool ResetEncoders();
	bool ReadVersion(char *version);
	uint16_t ReadMainBatteryVoltage(bool *valid = NULL);
	uint16_t ReadLogicBatteryVoltage(bool *valid = NULL);
	bool SetMinVoltageLogicBattery(uint8_t voltage);
	bool SetMaxVoltageLogicBattery(uint8_t voltage);
	bool SetM1VelocityPID(float Kp, float Ki, float Kd, uint32_t qpps);
	bool SetM2VelocityPID(float Kp, float Ki, float Kd, uint32_t qpps);
	uint32_t ReadISpeedM1(uint8_t *status = NULL, bool *valid = NULL);
	uint32_t ReadISpeedM2(uint8_t *status = NULL, bool *valid = NULL);
	bool DutyM1(uint16_t duty);
	bool DutyM2(uint16_t duty);
	bool DutyM1M2(uint16_t duty1, uint16_t duty2);
	bool SpeedM1(uint32_t speed);
	bool SpeedM2(uint32_t speed);
	bool SpeedM1M2(uint32_t speed1, uint32_t speed2);
	bool SpeedAccelM1(uint32_t accel, uint32_t speed);
	bool SpeedAccelM2(uint32_t accel, uint32_t speed);
	bool SpeedAccelM1M2(uint32_t accel, uint32_t speed1, uint32_t speed2);
	bool SpeedDistanceM1(uint32_t speed, uint32_t distance, uint8_t flag = 0);
	bool SpeedDistanceM2(uint32_t speed, uint32_t distance, uint8_t flag = 0);
	bool SpeedDistanceM1M2(uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag = 0);
	bool SpeedAccelDistanceM1(uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag = 0);
	bool SpeedAccelDistanceM2(uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag = 0);
	bool SpeedAccelDistanceM1M2(uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag = 0);
	bool ReadBuffers(uint8_t &depth1, uint8_t &depth2);
	bool ReadPWMs(int16_t &pwm1, int16_t &pwm2);
	bool ReadCurrents(int16_t &current1, int16_t &current2);
	bool SpeedAccelM1M2_2(uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2);
	bool SpeedAccelDistanceM1M2_2(uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag = 0);
	bool DutyAccelM1(uint16_t duty, uint32_t accel);
	bool DutyAccelM2(uint16_t duty, uint32_t accel);
	bool DutyAccelM1M2(uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2);
	bool ReadM1VelocityPID(float &Kp_fp, float &Ki_fp, float &Kd_fp, uint32_t &qpps);
	bool ReadM2VelocityPID(float &Kp_fp, float &Ki_fp, float &Kd_fp, uint32_t &qpps);
	bool SetMainVoltages(uint16_t min, uint16_t max);
	bool SetLogicVoltages(uint16_t min, uint16_t max);
	bool ReadMinMaxMainVoltages(uint16_t &min, uint16_t &max);
	bool ReadMinMaxLogicVoltages(uint16_t &min, uint16_t &max);
	bool SetM1PositionPID(float kp, float ki, float kd, uint32_t kiMax, uint32_t deadzone, uint32_t min, uint32_t max);
	bool SetM2PositionPID(float kp, float ki, float kd, uint32_t kiMax, uint32_t deadzone, uint32_t min, uint32_t max);
	bool ReadM1PositionPID(float &Kp, float &Ki, float &Kd, uint32_t &KiMax, uint32_t &DeadZone, uint32_t &Min, uint32_t &Max);
	bool ReadM2PositionPID(float &Kp, float &Ki, float &Kd, uint32_t &KiMax, uint32_t &DeadZone, uint32_t &Min, uint32_t &Max);
	bool SpeedAccelDeccelPositionM1(uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag);
	bool SpeedAccelDeccelPositionM2(uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag);
	bool SpeedAccelDeccelPositionM1M2(uint32_t accel1, uint32_t speed1, uint32_t deccel1, uint32_t position1, uint32_t accel2, uint32_t speed2, uint32_t deccel2, uint32_t position2, uint8_t flag);
	bool SetM1DefaultAccel(uint32_t accel);
	bool SetM2DefaultAccel(uint32_t accel);
	bool SetPinFunctions(uint8_t S3mode, uint8_t S4mode, uint8_t S5mode);
	bool GetPinFunctions(uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode);
	bool SetDeadBand(uint8_t Min, uint8_t Max);
	bool GetDeadBand(uint8_t &Min, uint8_t &Max);
	bool ReadEncoders(uint32_t &enc1, uint32_t &enc2);
	bool ReadISpeeds(uint32_t &ispeed1, uint32_t &ispeed2);
	bool RestoreDefaults();
	bool ReadTemp(uint16_t &temp);
	bool ReadTemp2(uint16_t &temp);
	uint32_t ReadError(bool *valid = NULL);
	bool ReadEncoderModes(uint8_t &M1mode, uint8_t &M2mode);
	bool SetM1EncoderMode(uint8_t mode);
	bool SetM2EncoderMode(uint8_t mode);
	bool WriteNVM();
	bool ReadNVM();
	bool SetConfig(uint16_t config);
	bool GetConfig(uint16_t &config);
	bool SetM1MaxCurrent(uint32_t max);
	bool SetM2MaxCurrent(uint32_t max);
	bool ReadM1MaxCurrent(uint32_t &max);
	bool ReadM2MaxCurrent(uint32_t &max);
	bool SetPWMMode(uint8_t mode);
	bool GetPWMMode(uint8_t &mode);

	int available();
	void begin();
	int read(uint32_t timeout = 0);
	size_t write(uint8_t byte);
	void flush();
	void clear();

private:
	void crc_clear();
	void crc_update(uint8_t data);
	uint16_t crc_get();
	bool write_n(uint8_t byte, ...);
	bool read_n(uint8_t byte, uint8_t cmd, ...);
	uint32_t Read4_1(uint8_t cmd, uint8_t *status, bool *valid);
	uint32_t Read4(uint8_t cmd, bool *valid);
	uint16_t Read2(uint8_t cmd, bool *valid);
	uint8_t Read1(uint8_t cmd, bool *valid);
};