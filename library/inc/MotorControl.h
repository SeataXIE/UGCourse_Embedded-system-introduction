#ifndef __MotorControl_H
#define __MotorControl_H

#include "stm32f10x_lib.h"

#define FORWARDA           1
#define BACKWARDA         -1

#define FORWARDB           1
#define BACKWARDB         -1

#define LeftWheel          0xF1
#define RightWheel         0xF2
#define BothWheel          0xF3

#define MaxPulsePer10ms  15

extern volatile unsigned int Count10msNum;
//extern volatile unsigned int Count10msNumPID;
extern volatile signed int MotorADesireSpeed;
//注意：在首次更新MotorDesireSpeed时，必须同时更新MotorLastDirection
//extern volatile signed int MotorCurrentSpeed;
extern volatile signed char MotorALastDirection;
extern volatile signed char MotorACurrentDirection;

extern volatile signed long Pulse_A;
extern volatile signed long Distance;

extern volatile u8 motorA_flag ;


extern void SetSpeed(unsigned char wheel, signed int Speed);
extern signed char GetDirectionA(void);
extern signed char GetDirectionB(void);
extern void Forward_Cmd(void);
extern void Backward_Cmd(void);
extern void Turn_Right(void);
extern void Turn_Left(void);
extern void Break_Cmd(void);


#endif
