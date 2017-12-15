#include "MotorControl.h"



//        PB12--IN1    PB13--IN2      ����
#define MotorBackwardA   {GPIO_ResetBits(GPIOB,GPIO_Pin_12);GPIO_SetBits(GPIOB,GPIO_Pin_13);}
#define MotorForwardA  {GPIO_SetBits(GPIOB,GPIO_Pin_12);GPIO_ResetBits(GPIOB,GPIO_Pin_13);}
#define MotorBrakeA      {GPIO_SetBits(GPIOB,GPIO_Pin_12);GPIO_SetBits(GPIOB,GPIO_Pin_13);}
#define DirectionA      GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)  // 1��ʾ���ˣ�0��ʾǰ��

//      PB14--IN3     PB15--IN4       ����
#define MotorBackwardB   {GPIO_SetBits(GPIOB,GPIO_Pin_14);GPIO_ResetBits(GPIOB,GPIO_Pin_15);}
#define MotorForwardB    {GPIO_ResetBits(GPIOB,GPIO_Pin_14);GPIO_SetBits(GPIOB,GPIO_Pin_15);}
#define MotorBrakeB      {GPIO_SetBits(GPIOB,GPIO_Pin_14);GPIO_SetBits(GPIOB,GPIO_Pin_15);}


volatile signed long Distance = 0;

volatile unsigned int Count10msNum = 0;
//volatile unsigned int Count10msNumPID =0;
volatile signed int MotorADesireSpeed = 0;
//ע�⣺���״θ���MotorDesireSpeedʱ������ͬʱ����MotorLastDirection
volatile signed char MotorALastDirection = 0;
volatile signed char MotorACurrentDirection = 0;
volatile signed long Distance_A = 0;
volatile signed long Pulse_A = 0;
volatile signed long LastPulse_A = 0;
volatile u8 motorA_flag = 1;

volatile signed int MotorBDesireSpeed = 0;
//ע�⣺���״θ���MotorDesireSpeedʱ������ͬʱ����MotorLastDirection
volatile signed char MotorBLastDirection = 0;
volatile signed char MotorBCurrentDirection = 0;
volatile signed long Distance_B = 0;
volatile signed long Pulse_B = 0;
volatile signed long LastPulse_B = 0;
u8 sp=50;

void SetSpeed(unsigned char wheel, signed int Speed)
{
  unsigned int TempSpeed;
  switch(wheel)
  {
  case RightWheel:
    //MotorADesireSpeed = Speed;
    // locationPID_A.LocationRef = MotorADesireSpeed*MaxPulsePer10ms/100;
    if( Speed>0 )
    {						  
      if( Speed>=100 ) Speed = 100; //���������ٷ�Χ100,�������Ϊ100
      TempSpeed = (unsigned int)(Speed*5);
      TIM3->CCR3 = TempSpeed;
      MotorForwardA;       //�����ת		
    }
    else if( Speed<0 )
    {							
      if( Speed<=(-100) ) Speed = -100;//���������ٷ�Χ-100,�������Ϊ-100
      TempSpeed = (unsigned int)((-1)*Speed*5);
      TIM3->CCR3 = TempSpeed;
      MotorBackwardA;	   	 	   			//�����ת	
    }
    else
    {
      TIM3->CCR3=0;   
      MotorBrakeA;
    }
    break;
    
  case LeftWheel:
    if( Speed>0 )
    {						  
        if( Speed>=100 ) Speed = 100; //���������ٷ�Χ100,�������Ϊ100
        TempSpeed = (unsigned int)(Speed*5);
        TIM3->CCR4 = TempSpeed;
        MotorForwardB;       //�����ת		
    }
    else if( Speed<0 )
    {							
        if( Speed<=(-100) ) Speed = -100;//���������ٷ�Χ-100,�������Ϊ-100
        TempSpeed = (unsigned int)((-1)*Speed*5);
        TIM3->CCR4 = TempSpeed;
        MotorBackwardB;	   	 	   			//�����ת	
    }
    else
    {
        TIM3->CCR4 = 0;
        MotorBrakeB;
    }
    break;

  case BothWheel:
     if( Speed>0 )
     {						  
        if( Speed>=100 ) Speed = 100; //���������ٷ�Χ100,�������Ϊ100
        TempSpeed = (unsigned int)(Speed*5);                       
        TIM3->CCR3 = TempSpeed;
        MotorForwardA;       	
        TIM3->CCR4 = TempSpeed;
        MotorForwardB;    
     }
     else if( Speed<0 )
     {							
        if( Speed<=(-100) ) Speed = -100;//���������ٷ�Χ-100,�������Ϊ-100
        TempSpeed = (unsigned int)((-1)*Speed*5);
        TIM3->CCR3 = TempSpeed;
        MotorBackwardA;       	
        TIM3->CCR4 = TempSpeed;
        MotorBackwardB;    
     }
     else
     {  
       TIM3->CCR3 = 0;
       TIM3->CCR4 = 0;
       MotorBrakeA;
       MotorBrakeB;
     }
    break;
  } 
}

void Forward_Cmd(void)
{
  SetSpeed(BothWheel,50);
}


void Backward_Cmd(void)
{
  SetSpeed(BothWheel,-50);
}

void Turn_Right(void)
{
  SetSpeed(LeftWheel,-30);
  SetSpeed(RightWheel,50);
}

void Turn_Left(void)
{
  SetSpeed(LeftWheel,50);
  SetSpeed(RightWheel,-30);
}


void Break_Cmd(void)
{
  TIM3->CCR3 = 0;
  TIM3->CCR4 = 0;
  MotorBrakeA;
  MotorBrakeB;
}

void circle_Left(void)
{
   SetSpeed(LeftWheel,50);
   SetSpeed(RightWheel,95);
}
void circle_Right(void)
{
   SetSpeed(LeftWheel,95);
   SetSpeed(RightWheel,50);
}
void nishizhen(float r)
{
            
  TIM3->CCR3 = 0;
  TIM3->CCR4 = 0;
  if(Distance_A<=2*3.14*(r-62))
  {
    dist=(r+62)/(r-62)*Distance_A;
    if(Distance_B != dist)
    {
      if(Distance_B>dist)
         circle_Right();
      if(Distance_B<dist)
         circle_Left();
    }
    else 
      circle_Left();
  }
   else
     Break_Cmd();
}
        
void shunshizhen(float r)
{
  TIM3->CCR3 = 0;
  TIM3->CCR4 = 0;
  if(Distance_B<=2*3.14*(r-62))
  {
    dist=(r+62)/(r-62)*Distance_B;
    if(Distance_A != dist)
    {
      if(Distance_A>dist)
         circle_Left();
      if(Distance_A<dist)
         circle_Right();
    }
    else 
      circle_Right();
  }
  else
    Break_Cmd();
}

void speedup(void)
{ 
  if(sp<=100)
  {
     sp+=5;
     SetSpeed(BothWheel,sp);
     Delay(0x1FFFFF);
   }
}
     
void speeddown(void)
{ 
  if(sp>=0)
  {
     sp-=5;
     SetSpeed(BothWheel,sp);
     Delay(0x1FFFFF);
   }
}

void yuan (void)
{
  shunshizhen(200);
}

void bizhang (void)
{
  if(ADC_Ch0_Res<30000||ADC_Ch3_Res<30000)
  {
    Backward_Cmd();//ǰ���У�����
    Delay(0x1FFFFF);
    if(ADC_Ch1_Res<30000&&ADC_Ch2_Res<30000)
    {
      Backward_Cmd();//ǰ���У�����
      Delay(0x9FFFFF); 
    }//������
    if(ADC_Ch1_Res<30000&&ADC_Ch2_Res>30000)
    {
      Turn_Right();
      Delay(0x9FFFFF);
    }//ǰ������У��ұ�û��
    if(ADC_Ch1_Res>30000&&ADC_Ch2_Res<30000)
    {
      Turn_Left();
      Delay(0x9FFFFF);
    }//ǰ���ұ��У����û��
    if(ADC_Ch1_Res>30000&&ADC_Ch2_Res>30000)
    {
      Turn_Left();
      Delay(0x9FFFFF);
    }//ǰ���У�����û��
  }
  else
  {
    Forward_Cmd();
  }
} 
void xunxian(void)
{
  if(ADC_Ch0_Res<30000||ADC_Ch3_Res<30000)Break_Cmd();
  else 
  {
    if(ADC_Ch1_Res>30000&&ADC_Ch2_Res<30000){Turn_Left();Delay(0xFFFF);}
    else 
    {
      if(ADC_Ch1_Res<30000&&ADC_Ch2_Res>30000){Turn_Right();Delay(0xFFFF);}
      else 
      {
        if(ADC_Ch1_Res<30000&&ADC_Ch2_Res<30000) Forward_Cmd();
        else Backward_Cmd(); 
      }
    }
  }
}

             

/*********************************************************************
 * Function:        signed char GetDirection(void)
 *
 * PreCondition:    )
 *
 * Input:          void
 *
 * Output:          None
 *
 * Overview:       ��ȡ�������
 *                  
 * Note:            None
 ********************************************************************/
signed char GetDirectionA(void)
{
  //�� PB5���ţ��жϵ������ת
  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5))
    return BACKWARDA;
  else
    return FORWARDA;
}

signed char GetDirectionB(void)
{
  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7))
    return FORWARDB;
  else
    return BACKWARDB;
}



/*******************************************************************************
* Function Name  : TIM1_UP_IRQHandler
* Description    : This function handles TIM1 overflow and update interrupt 
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_UP_IRQHandler(void)   //0.01s = 10ms
{
  //[ÿ10ms �ɼ�һ��TIM2 ��������]
  MotorALastDirection = MotorACurrentDirection;   
  MotorACurrentDirection = GetDirectionA();   //������ֵ��A��ת��
  if(MotorACurrentDirection == 1)             
    Pulse_A += TIM2->CNT;   //���ҵ���Ĵ�ŵ�Pulse_A ��
  else
    Pulse_A -= TIM2->CNT;
  TIM2->CNT = 0;   
 // ÿһ�����壬С���н�0.145mm.������С����һ�����������н���·�̡�
  Distance_A = Pulse_A * 0.145; 
  
  TIM1_ClearFlag(TIM1_FLAG_Update); 
  
  MotorBLastDirection = MotorBCurrentDirection;   
  MotorBCurrentDirection = GetDirectionB();   //������ֵ��A��ת��
  if(MotorBCurrentDirection == 1)             
    Pulse_B += TIM4->CNT;   //���ҵ���Ĵ�ŵ�Pulse_A ��
  else
    Pulse_B -= TIM4->CNT;
  TIM4->CNT = 0;   
 // ÿһ�����壬С���н�0.145mm.������С����һ�����������н���·�̡�
  Distance_B = Pulse_B * 0.145; 
  
  TIM1_ClearFlag(TIM1_FLAG_Update);
}
