#ifndef _HEALTHMANAGE_H
#define _HEALTHMANAGE_H
 
#include "stdint.h"

typedef struct{
	uint16_t 	prjID;				//��Ʒ��ʶ0xB012
	uint16_t    localMsge;			//λ����Ϣ0xFFFF
	uint8_t 	prjCode[10];		//��Ʒ����

}POWERBOARDMESSAGEX;
extern POWERBOARDMESSAGEX powerBoardMsgeX;


//Table_3
#pragma pack(1)
typedef struct{
	uint16_t 	prjID;				//��Ʒ��ʶ0xB012
	uint16_t    localMsge;			//λ����Ϣ0xFFFF
	uint8_t 	prjCode[10];		//��Ʒ����
	uint8_t	    unitCode[2];   		//������
	uint32_t	prjNum; 			//��Ʒ���
	uint8_t	    factorNume[8];		//������ţ���������α�ʶ�����κţ�˳��Ÿ�ռ2�ֽ�
	uint16_t	cpuModel;			//cpu�ͺ�
	uint16_t 	softNum;			//�������
	uint8_t 	reserve[8];			//����

}POWERBOARDMESSAGE;
extern POWERBOARDMESSAGE  	powerBoardMsge;

//Table_2
#pragma pack(1)
typedef struct{
	uint16_t 	prjID;				//��Ʒ��ʶ0xB011
	uint16_t    localMsge;			//λ����Ϣ0xFFFF
	uint8_t 	prjCode[10];		//��Ʒ����
	uint8_t	    unitCode[2];   		//������
	uint32_t	prjNum; 			//��Ʒ���
	uint8_t	    factorNume[8];		//������ţ���������α�ʶ�����κţ�˳��Ÿ�ռ2�ֽ�
	uint16_t	cpuModel;			//cpu�ͺ�
	uint16_t 	softNum;			//�������

	uint8_t 	APsoftCode[20];		//Ӧ��������������
	uint8_t	    APsoftVer[4];		//����汾��Ϣ����汾1-4(1byte),�м�汾A-Z(1byte),С�汾00-99(2bytes)
	uint16_t	APsoftDate;			//����̻�����

	uint8_t	    BMSsoftCode[20];	//BMS���������������
	uint8_t	    BMSsoftVer[4];		//����汾��Ϣ����汾1-4(1byte),�м�汾A-Z(1byte),С�汾00-99(2bytes)
	uint16_t 	BMSsoftDate;		//����̻�����

	uint32_t 	workTimeSum;		//�ۼƹ���ʱ��s
	uint32_t    workTime;			//���ι���ʱ��s
	uint32_t 	addPowCount;		//�ۼƼӵ����

	uint16_t 	prjTemp;			//����¶ȣ���Ʒ�ڲ���������¶���Ϣ
	uint16_t    cpuTemp;			//cpuоƬ�¶�
	uint16_t    ramSize;			//�ڴ��С
	uint16_t    flashSize;			//Flash��С
	uint16_t	flashUseRate;		//flash��С
	uint8_t	    ramUseRate;			//ramʹ����
	uint8_t	    cpuUseRate;			//cpuʹ����

	uint16_t 	voltP5V;
	uint16_t 	currentP5A;
	uint16_t 	voltP3V3;
	uint16_t 	voltP1V8;
	uint16_t 	voltP1V5;
	uint16_t 	voltP1V2;
	uint16_t 	voltP1V0;

	uint8_t 	reserve[8];			//����
}DETECTBOARDMESSAGE;
extern DETECTBOARDMESSAGE detectBoardMsge;

//Table_1
#pragma pack(1)
typedef struct {
    uint16_t    head;
    uint8_t     addr;
    uint8_t     length;

	uint16_t    prjID;						//��Ʒ��ʶ0xB010
	uint16_t 	local;						//λ����Ϣ
	uint8_t		prjCode[10];				//��Ʒ����
	uint8_t    unitCode[2];				//�����̵�λ����
	uint32_t 	prjNum;						//��Ʒ���
	uint8_t		factorNum[8];				//������ţ���������α�ʶ�����κţ�˳��Ÿ�ռ2�ֽ�
	uint16_t	cpuModel;					//cpu�ͺ�
	uint16_t	softNum;					//�������
	uint16_t	belongPrjNum;				//������Ʒ����

	DETECTBOARDMESSAGE 	detectBoardMsge;	    //��Ʒ��Ϣ
	POWERBOARDMESSAGE  	powerBoardMsge;		    //״̬��Ϣ

	uint8_t		reserve[8];					//����
	uint8_t		custMsae[8];				//�Զ�����Ϣ
    
    uint8_t     crc;
}HEALTHMANAGE;
extern HEALTHMANAGE healthManage;

//updatePara cycle
#pragma pack(1)
typedef struct {
    uint16_t    head;
    uint8_t     addr;
    uint8_t     length;

	uint32_t 	workTimeSum;		//�ۼƹ���ʱ��s
	uint32_t    workTime;			//���ι���ʱ��s
	uint32_t 	addPowCount;		//�ۼƼӵ����
    
	uint16_t 	prjTemp;			//����¶ȣ���Ʒ�ڲ���������¶���Ϣ
	uint16_t    cpuTemp;			//cpuоƬ�¶�
	uint16_t    ramSize;			//�ڴ��С
	uint16_t    flashSize;			//Flash��С
	uint16_t	flashUseRate;		//flash��С
	uint8_t	    ramUseRate;			//ramʹ����
	uint8_t	    cpuUseRate;			//cpuʹ����

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