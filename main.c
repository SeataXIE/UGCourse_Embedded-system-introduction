/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V1.0
* Date               : 10/08/2007
* Description        : Main program body
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "math.h"
#include "stdio.h"
#include "MotorControl.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((u32)0x4001244C)

//ADC1 4001 2400H
//ADC_DR偏移地址 4CH

volatile u16 ADC_Ch0_Res, ADC_Ch1_Res, ADC_Ch2_Res, ADC_Ch3_Res;//四路红外传感器
volatile u16 ADC_RegularConvertedValueTab[4];
u32 ADC_Calibration_DR;
extern u8 flag;
extern volatile signed long Distance_A ;
extern volatile signed long Distance_B ;
u16 dist=0;
extern u8 temp;



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
USART_InitTypeDef USART_InitStructure;
TIM1_TimeBaseInitTypeDef  TIM1_TimeBaseStructure;
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;


ErrorStatus HSEStartUpStatus;

#define FILTER_SAMPLES 16

u16 ADC_Value[4];


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//TIM_OCInitTypeDef  TIM_OCInitStructure;



/* Private function prototypes -----------------------------------------------*/

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration(void);
void TIM_Configuration(void);
void DMA_Configuration(void);
void ADC_Configuration(void);
void EXTI_Configuration(void);
void Delay(u32 nTime);   /*实现准确ms级准确延时*/


//u16  ADC_Filter(u16 *ADC_Value);
//u16  ADC_GetVolt(void);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void main(void)
{
  #ifdef DEBUG
    debug();
  #endif

  /* System Clocks Configuration */
  RCC_Configuration();

  /* NVIC configuration */
  NVIC_Configuration();
  
  EXTI_Configuration();
  /* Configure the GPIO ports */
  GPIO_Configuration();

  /* USART1 configuration ------------------------------------------------------*/
  USART_Configuration();
  
  TIM_Configuration();

  /* DMA Channel1 Configuration ----------------------------------------------*/
  DMA_Configuration();
  
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_Configuration();
  
  SysTick_SetReload(9000);
  SysTick_ITConfig(ENABLE);


  USART_SendData(USART1,(0xAA));
  printf("    /************************************************************************/");
  printf("\n");Delay(0x64);
  printf("      /*                        程序说明                                        */");
  printf("\n");Delay(0x64);
  printf("      /*    1、限制行进速度：0-100；                                            */");
  printf("\n");Delay(0x64);
  printf("      /*    2、+：加速；-：减速；                                               */");
  printf("\n");Delay(0x64);
  printf("      /*    3、输入x、b、y，分别进行巡线、避障、画圆；                               */");
  printf("\n");Delay(0x64);
  printf("      /*    4、输入w、a、s、d分别进行前进、左转、后退、右转控制、空格刹车；     */");
  printf("\n");Delay(0x64);
  printf("      /************************************************************************/\n");
  printf("\n");
  printf("enter:");
  
  while(1)
  {
    
   
     //printf("\r\n Distance_A:  %d",Distance_A);
     //printf("\r\n Distance_B: %d",Distance_B); 
    ADC_Ch0_Res = ADC_RegularConvertedValueTab[0] - ADC_Calibration_DR;
    ADC_Ch1_Res = ADC_RegularConvertedValueTab[1] - ADC_Calibration_DR;
    ADC_Ch2_Res = ADC_RegularConvertedValueTab[2] - ADC_Calibration_DR;
    ADC_Ch3_Res = ADC_RegularConvertedValueTab[3] - ADC_Calibration_DR; 

    

    if(temp==' ')  Break_Cmd(); 
    if(temp=='w')  Forward_Cmd();
    if(temp=='s')  Backward_Cmd();
    if(temp=='d')  Turn_Left();
    if(temp=='a')  Turn_Right(); 
    if(temp=='+')  speedup();  //加速
    if(temp=='-')  speeddown();//减速
    if(temp=='x')  xunxian();//巡线
    if(temp=='b')  bizhang ();//避障
    if(temp=='y')  yuan ();//画圆，半径20
    
  }
}
      

void USART_Configuration(void)
{
  /* USART1 configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the second edge
        - USART LastBit: The clock pulse of the last data bit is not output to
                         the SCLK pin
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_Clock = USART_Clock_Disable;
  USART_InitStructure.USART_CPOL = USART_CPOL_Low;
  USART_InitStructure.USART_CPHA = USART_CPHA_1Edge;
  USART_InitStructure.USART_LastBit = USART_LastBit_Disable;

    /* Configure the USART1 */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable USART1 */
  //USART_Cmd(USART1, ENABLE);
  
  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

/* Enable the USART Receive interrupt: this interrupt is generated when the 
   USART1 receive data register is not empty */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);
  
}


void DMA_Configuration(void)
{
  DMA_DeInit(DMA_Channel1);//复位开启DMA1的第一通道
 
  //DMA对应的外设基地址
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  
  //序列1转换结果放在ADC_RegularConvertedValueTab[0],序列2……
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_RegularConvertedValueTab;  
  //DMA的转换模式：SRC模式，从外设向内存中传送数据
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 4;
  //BufferSize = 4，ADC转换序列有4个通道
  
  //接收一次数据后，设备地址是否后移
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  //接收一次数据后，目标内存地址自动后移，用来采集多个数据
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
 
  //转换结果的数据大小  
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  
 //转换模式：常用循环缓存模式。Buffer写满后，自动回到初始地址开始传输
  //另外一种Normal模式：不循环，仅一次DMA
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA优先级，高
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//内存到内存模式禁止
  DMA_Init(DMA_Channel1, &DMA_InitStructure);
  

  /* Enable DMA channel1 */
  DMA_Cmd(DMA_Channel1, ENABLE);
}

void ADC_Configuration(void)
{
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//每个ADC独立工作
  //ADC扫描所有规则转换通道ADC_SQRx和注入转换通道ADC_JSQR
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换模式开启
  
  /*关闭ADC外部触发，即禁止由外部触发模数转换*/
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//转换数据右对齐
  ADC_InitStructure.ADC_NbrOfChannel = 4;   //开启4个通道
  ADC_Init(ADC1, &ADC_InitStructure);  //调用固件库函数完成初始化
  
 
  /* ADC1 regular channel configuration */ //采样周期为239.5个周期
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_239Cycles5);
  
  
  
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);/*  使能ADC1  DMA*/

  /* Enable ADC1 ECO*/ //ADC转换完成中断使能
  //ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

    /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);// ADC1 复位校准
  
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1)); //等待校准寄存器初始化

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);//开始校准
  
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));//等到校准完成

  ADC_Calibration_DR = ADC1->DR;//保存校准码

  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);//软件启动ADC1进行连续转换
}


/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{																
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */

 // RCC_HSICmd(ENABLE);
  RCC_HSEConfig(RCC_HSE_ON);
 // RCC_LSEConfig(RCC_LSE_OFF);

  while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_9);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
   while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
    
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_USART3 | RCC_APB1Periph_I2C1,
                          ENABLE);

  /* Enable GPIOA  GPIOB USART1 GPIOC and ADC1 clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1
                         | RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1
                         | RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE); 
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);
}



/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Configure PA4 PA5 PA6 and PA7 
  (ADC Channel4 Channel5 Channel6 and Channel7) as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//红外传感的配置
  
 
  
  /* GPIOA Configuration:led1 Pin8 in Output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  //GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_RESET);
  
 
  /* GPIOC Configuration:led2 Pin13 in Output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
  

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;//key1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//key2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource11);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource2);
   
  /*GPIOB Configuration: TIM3 channel 3 and 4 */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用功能的推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  /* Configure Motor Foward or Backward or Stop */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13|GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

/* Configure Encoder1 and Key1(PA.1, 11) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Configure Direction1 Encoder2 Direction2 KEY2(PB.2 )as input floating */  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 |GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    // Enable ADC IRQChannel
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQChannel ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel =EXTI2_IRQChannel ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TIM_Configuration(void)
{

  u16 CCR3_Val = 450;
  u16 CCR4_Val = 450;
  
  /* TIM1 Peripheral Configuration */ 
  TIM1_DeInit();
  /* Time Base configuration */         //10kHz
  TIM1_TimeBaseStructure.TIM1_Prescaler = 7200;
  TIM1_TimeBaseStructure.TIM1_CounterMode = TIM1_CounterMode_Up;
  TIM1_TimeBaseStructure.TIM1_Period = 100; //10ms
  TIM1_TimeBaseStructure.TIM1_ClockDivision = 0;
  TIM1_TimeBaseStructure.TIM1_RepetitionCounter = 0;
  TIM1_TimeBaseInit(&TIM1_TimeBaseStructure); 
  
  TIM1_ClearFlag(TIM1_FLAG_Update);  
  TIM1_ITConfig(TIM1_IT_Update,ENABLE);  //使能定时器TIM1中断
   /* TIM1 counter enable */
  TIM1_Cmd(ENABLE);   //TIM1用于10ms的定时器
 

  /* -----------------------------------------------------------------------
  TIM3 Configuration: generate 2 PWM signals with 4 different duty cycles:
  TIM3CLK = 36 MHz, Prescaler = 10, TIM3 counter clock = 3600 KHz
  TIM3 ARR Register = 499 => TIM3 Frequency = TIM3 counter clock/(ARR + 1) 
  TIM3 Frequency = 7.2 KHz.

  TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
  TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5% 
----------------------------------------------------------------------- */
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 499;          
  TIM_TimeBaseStructure.TIM_Prescaler = 10;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;          
  TIM_OCInitStructure.TIM_Channel = TIM_Channel_3;          
  TIM_OCInitStructure.TIM_Pulse = 0;//CCR3_Val; //初始占空比值，可以设置为0 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
  TIM_OCInit(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_Channel = TIM_Channel_4;          
  TIM_OCInitStructure.TIM_Pulse = 0;//CCR4_Val;  
  TIM_OCInit(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);   //TIM3  输出两路PWM波，用于控制电机转速
  

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  /* Selects the TI2 as clock for TIM2: the external clock is
  connected to TI1 input pin, the rising edge is the active edge and
  no filter sampling is done (ICFilter = 0) */
  //[ 将TIM2的通道2 设为TIM2的外部时钟信号，
  //  TIM->CNT 的值即为通道2端输入脉冲数]
  TIM_TIxExternalClockConfig(TIM2, TIM_TIxExternalCLK1Source_TI2,TIM_ICPolarity_Rising, 0);  
  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);   //采集脉冲数
  
   /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  /* Selects the TI3 as clock for TIM2: the external clock is
  connected to TI3 input pin, the rising edge is the active edge and
  no filter sampling is done (ICFilter = 0) */
  TIM_TIxExternalClockConfig(TIM4, TIM_TIxExternalCLK1Source_TI1,TIM_ICPolarity_Rising, 0);
 
  /* TIM2 enable counter */
  TIM_Cmd(TIM4, ENABLE);     //采集脉冲数
  
  
}




void  EXTI_Configuration(void)
 {
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
 }

void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}


/*定义 fputc 此函数为printf所用*/
int fputc(int ch,FILE *f) 
{ 

   USART_SendData(USART1, (u8) ch);  
    /* Loop until the end of transmission */ 
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch; 
} 






#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}



#endif

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/

