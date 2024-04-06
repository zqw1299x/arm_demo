/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ʹ��RS232���ڣ��жϽ��ղ��ԣ���ʾ�������ݵ�buff�������м������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32 F429 ������
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "stm32f4xx.h"
#include "./usart/bsp_debug_usart.h"

uint8_t Rxflag=0;
uint8_t ucTemp;

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
	uint8_t ucaRxBuf[256];
	uint16_t usRxCount=0; 
	
  /*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
  Debug_USART_Config();
 
	/*����printf��������Ϊ�ض�����fputc��printf�����ݻ����������*/
	printf("\r\nPrintf��ʽ���������һ�������жϽ��ջ���ʵ�� \r\n");	

	
	/*�Զ��庯����ʽ*/
	Usart_SendString( RS232_USART, (uint8_t *)"�Զ��庯�����������һ�������жϽ��ջ���ʵ��\n" );
	Usart_SendString( RS232_USART, (uint8_t *)"�������ݲ��Իس�������\n" );
	
	/*STM32���ڽ��յ��ַ�������stm32f4xx_it.c�ļ����жϷ�������
	*���ո����ݣ������Rxflag��־λ��*/

  while(1)
	{	
		/* 
			����DEBUG_USART�ڵ����ݣ����������� 
			���Խ��˶δ����װΪһ�����������������������̵���
		*/
		if(Rxflag)
		{
			if (usRxCount < sizeof(ucaRxBuf))
			{
				ucaRxBuf[usRxCount++] = ucTemp;
			}
			else
			{
				usRxCount = 0;
			}
			
			/* �򵥵�ͨ��Э�飬�����س����з���Ϊ1������֡�������м������ж�ʵ���Զ������� */
			/* ���������ַ�����Ϊ���յ�һ������ */
			if (ucTemp == 0x0A)	/* �����ַ� */
			{		
				/*��⵽�лس��ַ��Ͱ����ݷ��ظ���λ��*/
				Usart_SendStr_length( RS232_USART, ucaRxBuf, usRxCount );
				//Usart_SendString();
				usRxCount = 0;
			}
			Rxflag=0;
		}
	}	


}



/*********************************************END OF FILE**********************/

