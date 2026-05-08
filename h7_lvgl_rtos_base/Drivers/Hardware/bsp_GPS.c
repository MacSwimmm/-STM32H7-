#include "bsp_GPS.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os2.h" 

static T_GNGGA tgngga;
static T_GNTHS tgnths;
static T_AGRIC tagric;
static T_GPHPR tgphpr;

PT_GNGGA GetGNGGA(void) { return &tgngga; }
PT_GNTHS GetGNTHS(void) { return &tgnths; }
PT_AGRIC GetAGRIC(void) { return &tagric; }
PT_GPHPR GetGPHPR(void) { return &tgphpr; }

void AGRIC_Analy(PT_AGRIC ptAGRIC)
{
	tagric = *ptAGRIC;
}

void GPS_SendCmd(const char* p_cmd)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)p_cmd, strlen(p_cmd), 200);
}

void GPS_Init(void)
{
	uint8_t ucConfigStep = 1;
	while (ucConfigStep)
	{
		osDelay(500); 
		switch(ucConfigStep++)
		{
			case 1:break;
			case 2:GPS_SendCmd("unlog\r\n");break;
			case 3:GPS_SendCmd("mode base time 60\r\n");break;
			case 4:GPS_SendCmd("mode rover automotive\r\n");break;
			case 5:GPS_SendCmd("gpgga 1\r\n");break;
			case 6:GPS_SendCmd("gpths 1\r\n");break;
			case 7:GPS_SendCmd("agricb 1\r\n");break;
			case 8:GPS_SendCmd("gphpr 1\r\n");break;
			case 9:GPS_SendCmd("saveconfig\r\n");break;
			case 10:GPS_SendCmd("mode rover automotive\r\n");break;
			case 11:ucConfigStep = 0;break;
		}
	}
	osDelay(500);
}

static void GNGGA_Decode(char* str)
{
	char *data[16];
	unsigned char i;
	for(i = 0; i <14; i++)
	{
		data[i] = str;
		while(*str != (i == 13? '*':','))
		{
			if(*str++ == '\0') return;
		}
		*str ++ = '\0';
	}
	data[i] = str;
	tgngga.utc_time = atof(data[0]);
	tgngga.lat = atof(data[1]);
	tgngga.lat_dir = *data[2];
	tgngga.lon = atof(data[3]);
	tgngga.lon_dir = *data[4];
	tgngga.qf = *data[5] - '0';
	tgngga.sats = atoi(data[6]);
	tgngga.Hdop = atof(data[7]);
	tgngga.Alt = atof(data[8]);
	strcpy(tgngga.a_units, data[9]);
	tgngga.undulation = atof(data[10]);
	strcpy(tgngga.u_units, data[11]);
	tgngga.Age = atof(data[12]);
	strcpy(tgngga.stn_ID, data[13]);
	strcpy(tgngga.crc, data[14]);	
}

static void GNTHS_Decode(char *str)
{
	char *data[8];
	unsigned char i;
	for(i = 0; i < 2; i++)
	{
		data[i] = str;
		while(*str != (i == 1? '*':','))
		{
			if(*str++ == '\0') return;
		}
		*str ++ = '\0';
	}
	data[i] = str;
	tgnths.Heading = atof(data[0]);
	tgnths.Mode = atof(data[1]);
	strcpy(tgnths.crc, data[2]);	
}

static void GNHPR_Decode(char* str)
{
	char *data[16];
	unsigned char i;
	for(i = 0; i < 8; i++)
	{
		data[i] = str;
		while(*str != (i == 7? '*':','))
		{
			if(*str++ == '\0') return;
		}
		*str ++ = '\0';
	}
	data[i] = str;
	tgphpr.utc_time = atof(data[0]);
	tgphpr.Heading = atof(data[1]);
	tgphpr.gphpr_Pitch = atof(data[2]);
	tgphpr.gphpr_Roll = atof(data[3]);
	tgphpr.qf = *data[4] - '0';
	tgphpr.sats = atoi(data[5]);
	tgphpr.Age = atof(data[6]);
	strcpy(tgphpr.stn_ID, data[7]);
	strcpy(tgphpr.crc, data[8]);
}

static void Data_Decode(char* msg)
{
	if(msg[0] != '$') return;
	if(msg[3] == 'G' && msg[4] == 'G' && msg[5] == 'A')
		GNGGA_Decode(msg + 7);
	if(msg[3] == 'T' && msg[4] == 'H' && msg[5] == 'S')
		GNTHS_Decode(msg + 7);
	if (msg[3] == 'H' && msg[4] == 'P' && msg[5] == 'R')
		GNHPR_Decode(msg + 7);
}

unsigned char ucAGRIC[512] = {0};

void GPS_RxPro_HAL(uint8_t* pBuf, uint16_t Size)
{
	if(Size == 0) return;
	
	uint16_t i = 0;
	char *pHead = NULL;
    
	while(i < Size - 2)
	{
		if(pBuf[i] == 0xAA && pBuf[i + 1] == 0x44 && pBuf[i + 2] == 0xB5)
		{
			memcpy(ucAGRIC, &pBuf[i], (Size - i > 512) ? 512 : (Size - i));
			AGRIC_Analy((PT_AGRIC)ucAGRIC);
			break;
		}
        i++;
	}
    
	i = 0;
	while(i < Size)
	{
		if(pBuf[i] == '$')
			pHead = (char*)pBuf + i;
			
		if(i > 0 && pBuf[i-1] == '\r' && pBuf[i] == '\n' && pHead)
		{
			pBuf[i-1] = '\0';
			pBuf[i]   = '\0';
			Data_Decode(pHead);
			pHead = NULL;
		}
        i++;
	}
}
