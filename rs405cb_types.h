#ifndef _RS405CB_TYPES_H_
#define _RS405CB_TYPES_H_

static_assert(sizeof(short) == 2 * sizeof(char));
static_assert(sizeof(char) == 1);

typedef union
{
	struct
	{
		unsigned char L;
		unsigned char H;
	} BYTE;
	short SHORT;
	unsigned short USHORT;
} SHORT_DATA;

static_assert(sizeof(SHORT_DATA) == sizeof(short));

typedef union
{
	struct
	{
		/* Read Only */
		SHORT_DATA    ModelNumber;
		unsigned char FirmwareVersion;
		unsigned char reserved0;
	        /* ROM */
		unsigned char ServoID_RW;
		unsigned char Reverse_RW;
		unsigned char BaudRate_RW;
		unsigned char ReturnDelay_RW;
		SHORT_DATA    CWAngleLimit_RW;
		SHORT_DATA    CCWAngleLimit_RW;
	        SHORT_DATA    BootLoader_RO;
		SHORT_DATA    TemperatureLimit_RO;
		signed char   CWSpeedLimit_RW;
		signed char   CCWSpeedLimit_RW;
		signed char   CWTorqueLimit_RW;
		signed char   CCWTorqueLimit_RW;
		unsigned char Damper_RW;
		unsigned char reserved1;
		unsigned char TorqueInSilence_RW;
		unsigned char WarmUpTime_RW;
		unsigned char CWComplianceMargin_RW;
		unsigned char CCWComplianceMargin_RW;
		unsigned char CWComplianceSlope_RW;
		unsigned char CCWComplianceSlope_RW;
		SHORT_DATA    PUNCH_RW;
	} DATA;
	unsigned char BYTE[30];
} ROM;

static_assert(sizeof(ROM) == 30);

typedef union
{
#pragma pack (1)
	struct
	{
		SHORT_DATA    GoalPosition_RW;
		SHORT_DATA    GoalTime_RW;
		unsigned char reserved0;
		unsigned char MaxTorque_RW;
		unsigned char TorqueEnable_RW;
		SHORT_DATA    GoalSpeed_RW;
		SHORT_DATA    GoalTorque_RW;
		unsigned char CascadeEnable_RW;
		SHORT_DATA    PresentPosition_RO;
		SHORT_DATA    PresentTime_RO;
		SHORT_DATA    PresentSpeed_RO;
		SHORT_DATA    PresentCurrentOrTorque_RO;
		SHORT_DATA    PresentTemperature_RO;
		SHORT_DATA    PresentVolts_RO;
		unsigned char reserved6;
		unsigned char reserved7;
		unsigned char reserved8;
		unsigned char reserved9;
		unsigned char reserved10;
		unsigned char reserved11;
		} DATA;
#pragma pack (0)
	unsigned char BYTE[30];
} RAM;

static_assert(sizeof(RAM) == 30);

typedef union
{
	struct 
	{
		ROM Rom;
		RAM Ram;
	} DATA;
	unsigned char BYTE[60];
} RS405CB_t;

static_assert(sizeof(RS405CB_t) == 60);

#endif /* _RS405CB_TYPES_H_ */


