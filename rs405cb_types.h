#ifndef _RS405CB_TYPES_H_
#define _RS405CB_TYPES_H_

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

typedef union
{
	struct
	{
		/* Read Only */
		SHORT_DATA    ModelNumber;
		unsigned char FirmwareVersion;
		unsigned char reserved;
	        /* ROM */
		unsigned char ServoID_RW;
		unsigned char Reverse_RW;
		unsigned char BaudRate_RW;
		unsigned char ReturnDelay_RW;
		SHORT_DATA    CWAngleLimit_RW;
		SHORT_DATA    CCWAngleLimit_RW;
		unsigned char reserved0;
		unsigned char reserved1;
		SHORT_DATA    TemperatureLimit_RO;
		unsigned char reserved2;
		unsigned char reserved3;
		unsigned char reserved4;
		unsigned char reserved5;
		unsigned char Damper_RW;
		unsigned char reserved6;
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

typedef union
{
	struct
	{
		SHORT_DATA    GoalPosition_RW;
		SHORT_DATA    GoalTime_RW;
		unsigned char reserved0;
		unsigned char MaxTorque_RW;
		unsigned char TorqueEnable_RW;
		unsigned char reserved1;
		unsigned char reserved2;
		unsigned char reserved3;
		unsigned char reserved4;
		unsigned char reserved5;
		SHORT_DATA    PresentPosition_RO;
		SHORT_DATA    PresentTime_RO;
		SHORT_DATA    PresentSpeed_RO;
		SHORT_DATA    PresentCurrent_RO;
		SHORT_DATA    PresentTemperature_RO;
		SHORT_DATA    PresentVolts_RO;
		unsigned char reserved6;
		unsigned char reserved7;
		unsigned char reserved8;
		unsigned char reserved9;
		unsigned char reserved10;
		unsigned char reserved11;
		} DATA;
	unsigned char BYTE[30];
} RAM;

typedef union
{
	struct 
	{
		ROM Rom;
		RAM Ram;
	} DATA;
	unsigned char BYTE[60];
} RS405CB_mem_t;

typedef union
{
        struct
        {
                unsigned char temperature_limit_error:1;
                unsigned char reserved1:1;
                unsigned char temperature_limit_alarm:1;
                unsigned char reserved2:1;
                unsigned char flash_ROM_write_error:1;
                unsigned char reserved3:1;
                unsigned char receive_packet_parse_error:1;
                unsigned char reserved4:1;
        } DATA;
        unsigned char BYTE;
} RS405CB_flags_t;

#endif /* _RS405CB_TYPES_H_ */


