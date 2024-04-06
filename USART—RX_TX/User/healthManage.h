#ifndef _HEALTHMANAGE_H
#define _HEALTHMANAGE_H
 
#include "stdint.h"

typedef struct{
	uint16_t 	prjID;				//产品标识0xB012
	uint16_t    localMsge;			//位置信息0xFFFF
	uint8_t 	prjCode[10];		//产品代号

}POWERBOARDMESSAGEX;
extern POWERBOARDMESSAGEX powerBoardMsgeX;


//Table_3
#pragma pack(1)
typedef struct{
	uint16_t 	prjID;				//产品标识0xB012
	uint16_t    localMsge;			//位置信息0xFFFF
	uint8_t 	prjCode[10];		//产品代号
	uint8_t	    unitCode[2];   		//制造商
	uint32_t	prjNum; 			//产品编号
	uint8_t	    factorNume[8];		//出厂编号：年代，批次标识，批次号，顺序号各占2字节
	uint16_t	cpuModel;			//cpu型号
	uint16_t 	softNum;			//软件数量
	uint8_t 	reserve[8];			//保留

}POWERBOARDMESSAGE;
extern POWERBOARDMESSAGE  	powerBoardMsge;

//Table_2
#pragma pack(1)
typedef struct{
	uint16_t 	prjID;				//产品标识0xB011
	uint16_t    localMsge;			//位置信息0xFFFF
	uint8_t 	prjCode[10];		//产品代号
	uint8_t	    unitCode[2];   		//制造商
	uint32_t	prjNum; 			//产品编号
	uint8_t	    factorNume[8];		//出厂编号：年代，批次标识，批次号，顺序号各占2字节
	uint16_t	cpuModel;			//cpu型号
	uint16_t 	softNum;			//软件数量

	uint8_t 	APsoftCode[20];		//应用软件，软件代号
	uint8_t	    APsoftVer[4];		//软件版本信息：大版本1-4(1byte),中间版本A-Z(1byte),小版本00-99(2bytes)
	uint16_t	APsoftDate;			//软件固化日期

	uint8_t	    BMSsoftCode[20];	//BMS监控软件，软件代号
	uint8_t	    BMSsoftVer[4];		//软件版本信息：大版本1-4(1byte),中间版本A-Z(1byte),小版本00-99(2bytes)
	uint16_t 	BMSsoftDate;		//软件固化日期

	uint32_t 	workTimeSum;		//累计工作时长s
	uint32_t    workTime;			//本次工作时间s
	uint32_t 	addPowCount;		//累计加电次数

	uint16_t 	prjTemp;			//最高温度，产品内测量的最高温度信息
	uint16_t    cpuTemp;			//cpu芯片温度
	uint16_t    ramSize;			//内存大小
	uint16_t    flashSize;			//Flash大小
	uint16_t	flashUseRate;		//flash大小
	uint8_t	    ramUseRate;			//ram使用率
	uint8_t	    cpuUseRate;			//cpu使用率

	uint16_t 	voltP5V;
	uint16_t 	currentP5A;
	uint16_t 	voltP3V3;
	uint16_t 	voltP1V8;
	uint16_t 	voltP1V5;
	uint16_t 	voltP1V2;
	uint16_t 	voltP1V0;

	uint8_t 	reserve[8];			//保留
}DETECTBOARDMESSAGE;
extern DETECTBOARDMESSAGE detectBoardMsge;

//Table_1
#pragma pack(1)
typedef struct {
    uint16_t    head;
    uint8_t     addr;
    uint8_t     length;

	uint16_t    prjID;						//产品标识0xB010
	uint16_t 	local;						//位置信息
	uint8_t		prjCode[10];				//产品代号
	uint8_t    unitCode[2];				//制造商单位代号
	uint32_t 	prjNum;						//产品编号
	uint8_t		factorNum[8];				//出厂编号：年代，批次标识，批次号，顺序号各占2字节
	uint16_t	cpuModel;					//cpu型号
	uint16_t	softNum;					//软件数量
	uint16_t	belongPrjNum;				//所属产品数量

	DETECTBOARDMESSAGE 	detectBoardMsge;	    //产品信息
	POWERBOARDMESSAGE  	powerBoardMsge;		    //状态信息

	uint8_t		reserve[8];					//保留
	uint8_t		custMsae[8];				//自定义信息
    
    uint8_t     crc;
}HEALTHMANAGE;
extern HEALTHMANAGE healthManage;

//updatePara cycle
#pragma pack(1)
typedef struct {
    uint16_t    head;
    uint8_t     addr;
    uint8_t     length;

	uint32_t 	workTimeSum;		//累计工作时长s
	uint32_t    workTime;			//本次工作时间s
	uint32_t 	addPowCount;		//累计加电次数
    
	uint16_t 	prjTemp;			//最高温度，产品内测量的最高温度信息
	uint16_t    cpuTemp;			//cpu芯片温度
	uint16_t    ramSize;			//内存大小
	uint16_t    flashSize;			//Flash大小
	uint16_t	flashUseRate;		//flash大小
	uint8_t	    ramUseRate;			//ram使用率
	uint8_t	    cpuUseRate;			//cpu使用率

	uint16_t 	voltP5V;
	uint16_t 	currentP5A;
	uint16_t 	voltP3V3;
	uint16_t 	voltP1V8;
	uint16_t 	voltP1V5;
	uint16_t 	voltP1V2;
	uint16_t 	voltP1V0;

    uint8_t     crc;
}HEALTHUPDATECYCLE;
extern HEALTHUPDATECYCLE healthUpdatCycle;

void healthParaInit(void);
void healthParaCollect(void);


#endif