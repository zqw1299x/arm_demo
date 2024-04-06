/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   串口接发测试，串口接收到数据后马上回传。
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F429 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
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
    while(u1_tx_idle!=1);																				//查看串口1上次发送是否结束，只要硬件正常，一定发送完了（串口1 8.4m 串口2 115200，无论串口2收多快，两次收之间串口1一定发完了）。					
    u1_tx_idle=0;
    DMA_Cmd(DMA2_Stream7,ENABLE);
}

//串口111111111111111111111111111111111
//发送发送发送发送发送发送发送发送发送，DMA传输完成中断
void DMA2_Stream7_IRQHandler(void) 
{
	DMA_Cmd(DMA2_Stream7,DISABLE);
	DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);	
	DMA_SetCurrDataCounter(DMA2_Stream7,TX1_BUFF_SIZE);
	u1_tx_idle=1;																				//U1发送完数据
}

void MY_RX_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar0,u32 mar1, u16 ndtr)
{ 
    DMA_InitTypeDef  DMA_InitStructure;

    if((u32)DMA_Streamx>(u32)DMA2)                          //得到当前stream是属于DMA2还是DMA1
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);   //DMA2时钟使能 
        
    }else 
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);   //DMA1时钟使能 
    }
    DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}      //等待DMA可配置 

    /* 配置 DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                    //通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;         //DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = mar0;           //DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //外设到存储器模式
    DMA_InitStructure.DMA_BufferSize = ndtr;                //数据传输量 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //使用普通模式 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //外设突发单次传输  
    DMA_DoubleBufferModeConfig(DMA_Streamx,mar1, DMA_Memory_0);
    DMA_DoubleBufferModeCmd(DMA_Streamx, ENABLE);
    DMA_Init(DMA_Streamx, &DMA_InitStructure);                              //初始化DMA Stream
} 

void MY_TX_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar0, u16 ndtr)
{ 
    DMA_InitTypeDef  DMA_InitStructure;

    if((u32)DMA_Streamx>(u32)DMA2)                          //得到当前stream是属于DMA2还是DMA1
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);   //DMA2时钟使能 
        
    }else 
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);   //DMA1时钟使能 
    }
    DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}                      //等待DMA可配置 

    /* 配置 DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                                    //通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;                         //DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = mar0;                           //DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //外设到存储器模式
    DMA_InitStructure.DMA_BufferSize = ndtr;                                //数据传输量 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //使用普通模式 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //外设突发单次传输  
    DMA_Init(DMA_Streamx, &DMA_InitStructure);                              //初始化DMA Stream	
} 


//bound:波特率
//void uart1_init(u32 bound)
//{
//    //GPIO端口设置
//    GPIO_InitTypeDef GPIO_InitStructure;
//    USART_InitTypeDef USART_InitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;
// 
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);                //使能GPIOA时钟
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);               //使能USART1时钟

//    //串口1对应引脚复用映射
//    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);             //GPIOA9复用为USART1
//    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);            //GPIOA10复用为USART1

//    //USART1端口配置
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;             //GPIOA9与GPIOA10
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                        //复用功能
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	                //速度50MHz
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //推挽复用输出
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //上拉
//    GPIO_Init(GPIOA,&GPIO_InitStructure);                               //初始化PA9，PA10

//    //USART1 初始化设置
//    USART_InitStructure.USART_BaudRate = bound;                         //波特率设置
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;         //字长为8位数据格式
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;              //一个停止位
//    USART_InitStructure.USART_Parity = USART_Parity_No;                 //无奇偶校验位
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
//    USART_InitStructure.USART_Mode =  USART_Mode_Rx | USART_Mode_Tx;	                            //收发模式
//    USART_Init(USART1, &USART_InitStructure);                                       //初始化串口1

//    USART_ClearFlag(USART1, USART_FLAG_TC);
//    
//    /* 嵌套向量中断控制器组选择 */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//    //USART1 NVIC 配置
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                   //串口1中断通道
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;             //抢占优先级3
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		            //子优先级3
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			            //IRQ通道使能
//    NVIC_Init(&NVIC_InitStructure);	                                    //根据指定的参数初始化VIC寄存器
//    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                      //开启相关中断

////    //DMA接收通道中断配置
////    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;  
////    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;  
////    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;        
////    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
////    NVIC_Init(&NVIC_InitStructure);    

//    //DMA发送通道中断配置
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
//    //DMA_Cmd(DMA2_Stream5,ENABLE);			//先使能DMA接收后使能串口，防止出现接收上溢
//    USART_Cmd(USART1, ENABLE);              //使能串口1 
//}

//bound:波特率
void uart1_init(u32 bound)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);                //使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);               //使能USART1时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);             //GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);            //GPIOA10复用为USART1

    //USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;             //GPIOA9与GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                        //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	                //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //上拉
    GPIO_Init(GPIOA,&GPIO_InitStructure);                               //初始化PA9，PA10

    //USART1 初始化设置
    USART_InitStructure.USART_BaudRate = bound;                                     //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode =  USART_Mode_Tx;	                            //收发模式
    USART_Init(USART1, &USART_InitStructure);                                       //初始化串口1

    USART_ClearFlag(USART1, USART_FLAG_TC);

    //DMA接收通道中断配置
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);    

    //DMA发送通道中断配置
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
    DMA_Cmd(DMA2_Stream5,ENABLE);			//先使能DMA接收后使能串口，防止出现接收上溢
    USART_Cmd(USART1, ENABLE);              //使能串口1 
}


//串口1111111111111111111111111111111111111111111111111
//接收接收接收接收接收接收接收接收接收，DMA传输完成中断
void DMA2_Stream5_IRQHandler(void) 
{
    //DMA_Cmd(DMA2_Stream5,DISABLE);
    USART1->CR1 = USART1->CR1&0xfffffffB;	//一次接收完成，关闭接收，知道下次同步信号来，再打开接收
    DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);	
    if(DMA_GetCurrentMemoryTarget(DMA2_Stream5)==0)
    {
        uart1_rec_flag=2;					//先传MEMORY 1 flag=2 表示正在DMA MEM1 ,MEM2的数据可读
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

//void USART1_IRQHandler(void)                	            //串口2中断服务程序
//{
//	u8 Res;
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)   //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{
//		Res =USART_ReceiveData(USART1);                     //(USART1->DR);	//指令控制的方式;	
//        if(uart1_rec_flag=='X')         //参数配置的内容接收;
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
//				USART_Cmd(USART1, DISABLE);  //失能串口2
//                rec_len = 0;
//			}
//			return;
//		}
//		
//		//找到头并且加以区分;
//		if(rec_len == 0)
//		{
//			uart1_rec_flag = 1;
//			rec_len ++;
//			uart1_rx_buffer[0] = Res;
//		}
//		else if(rec_len == 1)
//		{
//			uart1_rx_buffer[1] = Res;
//			if((uart1_rx_buffer[0] == 0xCF) && (uart1_rx_buffer[0] == 0x1A))         //调试状态的参数指令接收头为0xCF1A;
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
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
    /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
    //Debug_USART_Config();
    uint8_t message_buffer[250],finishBuffer[50];
    uint8_t crc = 0;
    uint8_t readbuff[6] = {0xAA,0xFB,0x00,0x01,0x12,0x34};
    uint8_t readbuffx[6] = {0xFB,0xAA,0x00,0x01,0x12,0x34};
    int i = 0;
		Debug_USART_Config();
    //uart1_init(9600);    /* 发送一个字符串 */
    Usart_SendString( DEBUG_USART,"这是一个串口中断接收回显实验\n");
    printf("这是一个串口中断接收回显实验\n");   
    while(1)
    {	
				if(uart1_rec_flag==1)	//数据串口1读到数据------1中数据已经准备好，正在操作2
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
            USART_Cmd(USART1, ENABLE);  //使能串口1
        }
    }	
}



/*********************************************END OF FILE**********************/

