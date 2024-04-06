
#include "healthManage.h"
#include "string.h"

POWERBOARDMESSAGE  	powerBoardMsge;
DETECTBOARDMESSAGE  detectBoardMsge;
HEALTHMANAGE        healthManage;
HEALTHUPDATECYCLE   healthUpdatCycle;
POWERBOARDMESSAGEX powerBoardMsgeX;


void healthParaInit(void)
{
	memset(&healthManage,0,sizeof(healthManage));
    memset(&healthUpdatCycle,0,sizeof(healthUpdatCycle));
    
    powerBoardMsgeX.prjID = 0xaafb;
    memset(powerBoardMsgeX.prjCode,0xee,10);
    powerBoardMsgeX.localMsge = 0xeeff;

    healthManage.head                       = 0xAAFB;
    healthManage.addr                       = 0x00;
    healthManage.length                     = 220;
	healthManage.prjID						= 0xB010;									//产品标识
	healthManage.local						= 0xFFFF;
	memcpy(healthManage.prjCode,"MN-1A",5);
	memcpy(healthManage.unitCode,"Lj",2);
	healthManage.prjNum						= 0xFFFFFFFF;
	sprintf(healthManage.factorNum,"%s%d","230A01",0x0001);
	healthManage.cpuModel					= 0x00;
	healthManage.softNum						= 0;
	healthManage.belongPrjNum				= 2;

	healthManage.detectBoardMsge.prjID		= 0xB011;									//产品标识检测板
	healthManage.detectBoardMsge.localMsge	= 0xFFFF;
	memcpy(healthManage.detectBoardMsge.prjCode,"daiding",7);
	memcpy(healthManage.detectBoardMsge.unitCode,"Lj",2);
	healthManage.detectBoardMsge.prjNum		= 0xFFFFFFFF;
	sprintf(healthManage.detectBoardMsge.factorNume,"%s%d","230A01",0x0001);
	healthManage.detectBoardMsge.cpuModel	= 0x14;
	healthManage.detectBoardMsge.softNum		= 2;

	healthManage.powerBoardMsge.prjID		= 0xB012;									//产品标识电源板
	healthManage.powerBoardMsge.localMsge	= 0xFFFF;
	memcpy(healthManage.powerBoardMsge.prjCode,"daiding",7);
	memcpy(healthManage.powerBoardMsge.unitCode,"Lj",2);
	healthManage.powerBoardMsge.prjNum		= 0xFFFFFFFF;
	sprintf(healthManage.powerBoardMsge.factorNume,"%s%d","230A01",0x0001);
	healthManage.powerBoardMsge.cpuModel		= 0;
	healthManage.powerBoardMsge.softNum		= 2; 
}

void healthParaCollect(void)
{
    uint16_t adc128Value[6]     = {0};
    uint16_t adc_ina226Value[8] = {0};
    healthUpdatCycle.head       = 0xAAFB;
    healthUpdatCycle.addr       = 118;
    healthUpdatCycle.length     = 38;

}

