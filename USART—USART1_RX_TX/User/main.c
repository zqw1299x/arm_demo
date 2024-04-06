/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ���ڽӷ����ԣ����ڽ��յ����ݺ����ϻش���
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
#include "string.h"
//#include "common.h"
//#include "healthManage.h"

#define RX1_BUFF_SIZE   300
#define TX1_BUFF_SIZE   300
char uart1_rx_buffer[RX1_BUFF_SIZE];
char uart1_rx_buffer1[RX1_BUFF_SIZE];
char uart1_tx_buffer[TX1_BUFF_SIZE];
u8 uart1_rec_flag=0;
u16 rec_len=0;
u16 rec_ding_len=0;
u16 uart1To6Len = 0;
char u1_tx_idle=1;
extern uint8_t idleFlag;
extern uint8_t rxbuf[100];

void delay(long cnt)
{
		while(cnt--);
}

void uart1_send_u1_tx_buff()
{
    while(u1_tx_idle!=1);																				//�鿴����1�ϴη����Ƿ������ֻҪӲ��������һ���������ˣ�����1 8.4m ����2 115200�����۴���2�ն�죬������֮�䴮��1һ�������ˣ���					
    u1_tx_idle=0;
    DMA_Cmd(DMA2_Stream7,ENABLE);
}

//����111111111111111111111111111111111
//���ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͣ�DMA��������ж�
void DMA2_Stream7_IRQHandler(void) 
{
	DMA_Cmd(DMA2_Stream7,DISABLE);
	DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);	
	DMA_SetCurrDataCounter(DMA2_Stream7,TX1_BUFF_SIZE);
	u1_tx_idle=1;																				//U1����������
}

void MY_RX_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar0,u32 mar1, u16 ndtr)
{ 
    DMA_InitTypeDef  DMA_InitStructure;

    if((u32)DMA_Streamx>(u32)DMA2)                          //�õ���ǰstream������DMA2����DMA1
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);   //DMA2ʱ��ʹ�� 
        
    }else 
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);   //DMA1ʱ��ʹ�� 
    }
    DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}      //�ȴ�DMA������ 

    /* ���� DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                    //ͨ��ѡ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;         //DMA�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = mar0;           //DMA �洢��0��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //���赽�洢��ģʽ
    DMA_InitStructure.DMA_BufferSize = ndtr;                //���ݴ����� 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //���������ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�洢������ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݳ���:8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�洢�����ݳ���:8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //ʹ����ͨģʽ 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //����ͻ�����δ���  
    DMA_DoubleBufferModeConfig(DMA_Streamx,mar1, DMA_Memory_0);
    DMA_DoubleBufferModeCmd(DMA_Streamx, ENABLE);
    DMA_Init(DMA_Streamx, &DMA_InitStructure);                              //��ʼ��DMA Stream
} 

void MY_TX_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar0, u16 ndtr)
{ 
    DMA_InitTypeDef  DMA_InitStructure;

    if((u32)DMA_Streamx>(u32)DMA2)                          //�õ���ǰstream������DMA2����DMA1
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);   //DMA2ʱ��ʹ�� 
        
    }else 
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);   //DMA1ʱ��ʹ�� 
    }
    DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}                      //�ȴ�DMA������ 

    /* ���� DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                                    //ͨ��ѡ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;                         //DMA�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = mar0;                           //DMA �洢��0��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //���赽�洢��ģʽ
    DMA_InitStructure.DMA_BufferSize = ndtr;                                //���ݴ����� 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //���������ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�洢������ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݳ���:8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�洢�����ݳ���:8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //ʹ����ͨģʽ 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //����ͻ�����δ���  
    DMA_Init(DMA_Streamx, &DMA_InitStructure);                              //��ʼ��DMA Stream	
} 


//bound:������
//void uart1_init(u32 bound)
//{
//    //GPIO�˿�����
//    GPIO_InitTypeDef GPIO_InitStructure;
//    USART_InitTypeDef USART_InitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;
// 
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);                //ʹ��GPIOAʱ��
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);               //ʹ��USART1ʱ��

//    //����1��Ӧ���Ÿ���ӳ��
//    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);             //GPIOA9����ΪUSART1
//    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);            //GPIOA10����ΪUSART1

//    //USART1�˿�����
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;             //GPIOA9��GPIOA10
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                        //���ù���
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	                //�ٶ�50MHz
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //���츴�����
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //����
//    GPIO_Init(GPIOA,&GPIO_InitStructure);                               //��ʼ��PA9��PA10

//    //USART1 ��ʼ������
//    USART_InitStructure.USART_BaudRate = bound;                         //����������
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;         //�ֳ�Ϊ8λ���ݸ�ʽ
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;              //һ��ֹͣλ
//    USART_InitStructure.USART_Parity = USART_Parity_No;                 //����żУ��λ
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
//    USART_InitStructure.USART_Mode =  USART_Mode_Rx | USART_Mode_Tx;	                            //�շ�ģʽ
//    USART_Init(USART1, &USART_InitStructure);                                       //��ʼ������1

//    USART_ClearFlag(USART1, USART_FLAG_TC);
//    
//    /* Ƕ�������жϿ�������ѡ�� */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//    //USART1 NVIC ����
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                   //����1�ж�ͨ��
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;             //��ռ���ȼ�3
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		            //�����ȼ�3
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			            //IRQͨ��ʹ��
//    NVIC_Init(&NVIC_InitStructure);	                                    //����ָ���Ĳ�����ʼ��VIC�Ĵ���
//    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                      //��������ж�

////    //DMA����ͨ���ж�����
////    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;  
////    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;  
////    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;        
////    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
////    NVIC_Init(&NVIC_InitStructure);    

//    //DMA����ͨ���ж�����
//    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;  
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;  
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;        
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
//    NVIC_Init(&NVIC_InitStructure);    

//    //MY_RX_DMA_Config(DMA2_Stream5,DMA_Channel_4,(u32)&USART1->DR,(u32)uart1_rx_buffer,(u32)uart1_rx_buffer,RX1_BUFF_SIZE);
//    MY_TX_DMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)&uart1_tx_buffer[0],TX1_BUFF_SIZE);

//    //USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
//    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);

//    //DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
//    DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
//    //DMA_Cmd(DMA2_Stream5,ENABLE);			//��ʹ��DMA���պ�ʹ�ܴ��ڣ���ֹ���ֽ�������
//    USART_Cmd(USART1, ENABLE);              //ʹ�ܴ���1 
//}

//bound:������
void uart1_init(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);                //ʹ��GPIOAʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);               //ʹ��USART1ʱ��

    //����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);             //GPIOA9����ΪUSART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);            //GPIOA10����ΪUSART1

    //USART1�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;             //GPIOA9��GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                        //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	                //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //����
    GPIO_Init(GPIOA,&GPIO_InitStructure);                               //��ʼ��PA9��PA10

    //USART1 ��ʼ������
    USART_InitStructure.USART_BaudRate = bound;                                     //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode =  USART_Mode_Tx;	                            //�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure);                                       //��ʼ������1

    USART_ClearFlag(USART1, USART_FLAG_TC);

    //DMA����ͨ���ж�����
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);    

    //DMA����ͨ���ж�����
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);    

    MY_RX_DMA_Config(DMA2_Stream5,DMA_Channel_4,(u32)&USART1->DR,(u32)uart1_rx_buffer,(u32)uart1_rx_buffer1,RX1_BUFF_SIZE);
    MY_TX_DMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)uart1_tx_buffer,TX1_BUFF_SIZE);

    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);

    DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
    DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA2_Stream5,ENABLE);			//��ʹ��DMA���պ�ʹ�ܴ��ڣ���ֹ���ֽ�������
    USART_Cmd(USART1, ENABLE);              //ʹ�ܴ���1 
}


//����1111111111111111111111111111111111111111111111111
//���ս��ս��ս��ս��ս��ս��ս��ս��գ�DMA��������ж�
void DMA2_Stream5_IRQHandler(void) 
{
    //DMA_Cmd(DMA2_Stream5,DISABLE);
    USART1->CR1 = USART1->CR1&0xfffffffB;	//һ�ν�����ɣ��رս��գ�֪���´�ͬ���ź������ٴ򿪽���
    DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);	
    if(DMA_GetCurrentMemoryTarget(DMA2_Stream5)==0)
    {
        uart1_rec_flag=2;					//�ȴ�MEMORY 1 flag=2 ��ʾ����DMA MEM1 ,MEM2�����ݿɶ�
        memcpy(uart1_tx_buffer,uart1_rx_buffer1,300);
        //LED0=!LED0;
    }
    else		
    {
        uart1_rec_flag=1;		
        memcpy(uart1_tx_buffer,uart1_rx_buffer,300);
        //LED1=!LED1;
    }  
}

//void USART1_IRQHandler(void)                	            //����2�жϷ������
//{
//	u8 Res;
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)   //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
//	{
//		Res =USART_ReceiveData(USART1);                     //(USART1->DR);	//ָ����Ƶķ�ʽ;	
//        if(uart1_rec_flag=='X')         //�������õ����ݽ���;
//		{
//			uart1_rx_buffer[rec_len]=Res;
//            rec_len++;
//			if((uart1_rx_buffer[rec_len-2] == 0xA1) && (uart1_rx_buffer[rec_len-1] == 0xFC))
//			{
//				uart1_rec_flag='U';
//				//memcpy(uart6_tx_buffer,uart1_rx_buffer,rec_len+1);
//                memset(uart1_tx_buffer,0,1024);
//                memcpy(uart1_tx_buffer,uart1_rx_buffer,rec_len+1);
//                uart1To6Len = rec_len;
//				USART_Cmd(USART1, DISABLE);  //ʧ�ܴ���2
//                rec_len = 0;
//			}
//			return;
//		}
//		
//		//�ҵ�ͷ���Ҽ�������;
//		if(rec_len == 0)
//		{
//			uart1_rec_flag = 1;
//			rec_len ++;
//			uart1_rx_buffer[0] = Res;
//		}
//		else if(rec_len == 1)
//		{
//			uart1_rx_buffer[1] = Res;
//			if((uart1_rx_buffer[0] == 0xCF) && (uart1_rx_buffer[0] == 0x1A))         //����״̬�Ĳ���ָ�����ͷΪ0xCF1A;
//			{
//				uart1_rec_flag='T';
//				rec_len=2;
//			}
//            else if((uart1_rx_buffer[0] == 0xCF) && (uart1_rx_buffer[1] == 0x1A))
//            {
//                uart1_rec_flag='X';
//                rec_len=2;
//            }
//			else
//			{
//				uart1_rx_buffer[0] = uart1_rx_buffer[1];
//				rec_len = 1;
//				uart1_rec_flag=0;
//			}
//			
//		}
//	}
//} 

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{	
    /*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
    //Debug_USART_Config();
    uint8_t message_buffer[250],finishBuffer[50];
    uint8_t crc = 0;
    uint8_t readbuff[6] = {0xAA,0xFB,0x00,0x01,0x12,0x34};
    uint8_t readbuffx[6] = {0xFB,0xAA,0x00,0x01,0x12,0x34};
    int i = 0;
		Debug_USART_Config();
    //uart1_init(9600);    /* ����һ���ַ��� */
    Usart_SendString( DEBUG_USART,"����һ�������жϽ��ջ���ʵ��\n");
    printf("����һ�������жϽ��ջ���ʵ��\n");   
    while(1)
    {	
				if(uart1_rec_flag==1)	//���ݴ���1��������------1�������Ѿ�׼���ã����ڲ���2
				{
						uart1_rec_flag=0;
						uart1_send_u1_tx_buff();
						
				}
				else if(uart1_rec_flag==2)
				{
						uart1_rec_flag=0;
						uart1_send_u1_tx_buff();
				}


        if(uart1_rec_flag == 'U')
        {
            uart1_rec_flag=0;
            uart1_send_u1_tx_buff();
            //uart6_send_tx_buff();
            //uart2_send_tx_buff_len(uart2To6Len);
            USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1
        }
    }	
}



/*********************************************END OF FILE**********************/

