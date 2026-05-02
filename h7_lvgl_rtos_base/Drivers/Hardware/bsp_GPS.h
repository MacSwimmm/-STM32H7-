#ifndef __BSP__GPS_H
#define __BSP__GPS_H

#include "main.h"

typedef struct{
	double utc_time;
	double lat;
	char lat_dir;
	double lon;
	char lon_dir;
	unsigned char qf;
	unsigned char sats;
	float Hdop;
	float Alt;
	char a_units[8];
	float undulation;
	char u_units[8];
	float Age;
	char stn_ID[8];
	char crc[8];
}T_GNGGA, *PT_GNGGA;

typedef struct{
	float Heading;
	char Mode;
	char crc[8];
}T_GNTHS, *PT_GNTHS;

typedef struct{
	unsigned char Sync1;
	unsigned char Sync2;
	unsigned char Sync3;
	unsigned char CPUIdle;
	unsigned short MessageID;
	unsigned short MessageLength;
	unsigned char TimeRef;
	unsigned char TimeStat;
	unsigned short Wn;
	unsigned long Ms;
	unsigned long Version;
	unsigned char Reserved;
	unsigned char LeapSec;
	unsigned short DelayMs;
}T_BinHead, *PT_BinHead;

typedef struct{
	T_BinHead tASCIIHead;
	char GNSS[4];
	unsigned char length;
	unsigned char Year;
	unsigned char Month;
	unsigned char Day;
	unsigned char Hour;
	unsigned char Minute;
	unsigned char Second;
	unsigned char Postype;
	unsigned char HeadingStat;
	unsigned char NumGPSSta;
	unsigned char NumBDSSta;
	unsigned char NumGLOSta;
	float Baseline_N;
	float Baseline_E;
	float Baseline_U;
	float Baseline_NStd;
	float Baseline_EStd;
	float Baseline_UStd;
	float Heading;
	float agric_Pitch;
	float agric_Roll;
	float Speed;
	float VelocityOfNorth;
	float VelocityOfEast;
	float VelocityOfUp;
	float XigemaVx;
	float XigemaVy;
	float XigemaVz;
	double lat;
	double lon;
	double alt;
	double ECEFX;
	double ECEFY;
	double ECEFZ;
	float XigemaLat;
	float XigemaLon;
	float XigemaAlt;
	float XigemaECEFX;
	float XigemaECEFY;
	float XigemaECEFZ;
	double BaseLat;
	double BaseLon;
	double BaseAlt;
	double SecLat;
	double SecLon;
	double SecAlt;
	int GPSWeekSecond;
	float Diffage;
	float SpeedHeading;
	float Undulation;
	float RemainFloat3;
	float RemainFloat4;
	unsigned char NumGalSta;
	unsigned char SpeedType;
	unsigned char RemainChar3;
	unsigned char RemainChar4;
	unsigned int crc32;
}T_AGRIC, *PT_AGRIC;
	
typedef struct{
	double utc_time;
	float Heading;
	float gphpr_Pitch;
	float gphpr_Roll;
	unsigned char qf;
	unsigned char sats;
	float Age;
	char stn_ID[8];
	char crc[8];
}T_GPHPR, *PT_GPHPR;


void GPS_Init(void);
void GPS_SendCmd(const char* p_cmd);
void GPS_RxPro_HAL(uint8_t* pBuf, uint16_t Size);

PT_GNGGA GetGNGGA(void);
PT_GNTHS GetGNTHS(void);
PT_AGRIC GetAGRIC(void);
PT_GPHPR GetGPHPR(void);

#endif 
