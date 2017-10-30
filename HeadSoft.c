#include "main.h"


//Code - размер кода всего байты
//RO-data - константы в коде (Read Only Data)
//RW-data - переменные (Read Write Data)
//ZI-data - переменные (Zero-Initialized Data)

char Time[12]=""; //время
char Status[2]=""; //валидность
char SLatitude[16]="";  //Латитуда
char NS[3]="";                          // 
char SLongitude[12]="";         //Лонгитуда 
char EW[3]="";                          // 
char CourseTrue[10]="";                 // курс
char Data[12]="";                               //Дата
char SatCount[4]="";                    //используемых спутников
char AltitudaMSL[12]="";            //высота
char ViewSat[4];   //
char COG[8]="";                 //      
char COGstat[4]="";             //
char Speed[8]="";                       //скорость
char SpeedAlt[8]="";    //
char UNUSED[32]="";                     //мусорка, тут все данные, которые не нужны
char Knot[8]="";
char *const RMC[]={Time,Status,SLatitude,NS,SLongitude,EW,UNUSED,CourseTrue,Data,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
char *const GGA[]={UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,SatCount,UNUSED,AltitudaMSL,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
char *const GSV[]={UNUSED,UNUSED,ViewSat,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
char *const VTG[]={COG,COGstat, UNUSED,UNUSED,Knot,UNUSED,Speed,UNUSED,UNUSED,UNUSED};
unsigned char GLONAS_COUNT=0;
unsigned char GPS_COUNT=0;
volatile char DataDone=0;
unsigned char DataValid=0;


// PC5_1 PC6_2
// PC8_1 PB13_2
// PB14   PB15


// Y1 ось камеры

/*
  Full-Step Increments
	1 step = 2 pulse
  1 step = 1.8° 
	200 step = 360°
	1 pulse = 0.9°
	400 pulse = 360°
*/

//int angle_big = 360;
uint16_t angle = 0;
uint16_t pulse = 0;
unsigned char i = 0;


int PWM_Y1;
int Period_Y1 = 5000;

int PWM_Y2;
int Period_Y2 = 5000;

int PWM_Z;
int Period_Z = 5000;


uint16_t angle_Y1 = 0;
uint16_t angle_Y2 = 180;
uint16_t angle_Z = 180;

unsigned char RUN_Y1 = 0;
unsigned char RUN_Y2 = 0;
unsigned char RUN_Z = 0;

unsigned char up_down_Y1 = 0;
unsigned char up_down_Y2 = 0;
unsigned char up_down_Z = 0;

uint16_t pulse_Y1 = 0;
uint16_t pulse_Y2 = 0;
uint16_t pulse_Z = 0;

uint16_t tmppulse_Y1 = 0;
uint16_t tmppulse_Y2 = 0;
uint16_t tmppulse_Z = 0;

unsigned char motor_Y1 = 0;
unsigned char motor_Y2 = 2;
unsigned char motor_Z = 3;
uint8_t start = 1;
uint8_t end = 1;

uint8_t a = 0;
uint8_t b = 0;

uint8_t Y1 = 0;
uint8_t Y2 = 0;
uint8_t Z = 0;
uint8_t F = 1;
uint8_t Zero = 1;

uint8_t ZeroY1 = 1;
uint8_t ZeroY2 = 0;
uint8_t ZeroZ = 0;
uint8_t flag = 0, flag2 =0;

uint8_t startY1 = 1;
uint8_t startY2 = 1;
uint8_t startZ = 1;



uint16_t flowing_start_Y1 = 0;
uint16_t flowing_stop_Y1 = 0;
uint8_t pp_Y1 = 1;
uint8_t ps_Y1 = 0;

uint16_t flowing_start_Y2 = 0;
uint16_t flowing_stop_Y2 = 0;
uint8_t pp_Y2 = 1;
uint8_t ps_Y2 = 0;

uint16_t flowing_start_Z = 0;
uint16_t flowing_stop_Z = 0;
uint8_t pp_Z = 1;
uint8_t ps_Z = 0;

uint16_t step_Y1 = 0;
uint16_t step_Y2 = 0;
uint16_t step_Z = 0;



uint8_t Recieve_buf[6] = {0x81, 0, 0, 0, 0, 0xFF}; 
uint8_t buf[7] = {0, 0, 0, 0, 0, 0, 0};
uint8_t Recieve_W=0, Recieve_R=0, Recieve_C=0;

	/****************************************
	 *SystemFrequency/1000      1ms         *
	 *SystemFrequency/100000    10us        *
	 *SystemFrequency/1000000   1us         *
	 *****************************************/


void Parser(unsigned char data)
{
static unsigned char ByteCount=0xff;
static unsigned int MsgType;                                    
static char *MsgTxt=(char*)&MsgType;     
static  unsigned char ComaPoint=0xff;
static unsigned char CharPoint=0;
if(data=='$'){ByteCount=0;ComaPoint=0xff;MsgTxt=(char*)&MsgType; return;} //ждем начала стрки
if(ByteCount==0xff) return;                                                                     //
ByteCount++;
if(ByteCount<=1)        return;                                                         //
if(ByteCount<6&&ByteCount>1)            //берем 4 символа заголовка
        {
        *MsgTxt=data;   //и делаем из него число                                        
        MsgTxt++;
        return;
        }  
//
switch(MsgType)
        {
        case    0x434D5250:                             //GPRMC         
        case    0x434D524E:                             //GNRMC 
                if(data==',') {ComaPoint++;     CharPoint=0;RMC[ComaPoint][0]=0;return;}
                if(data=='*') {MsgType=0;return;}
                RMC[ComaPoint][CharPoint++]=data;
                RMC[ComaPoint][CharPoint]=0;
                return;
        case    0x41474750:                             //PGGA
        case    0x4147474e:                             //NGGA
                if(data==',')  {ComaPoint++;    CharPoint=0;GGA[ComaPoint][0]=0;return;}
                if(data=='*') {MsgType=0;return;}
                GGA[ComaPoint][CharPoint++]=data;
                GGA[ComaPoint][CharPoint]=0;
                return;
        case    0x47545650:             //PVTG
                if(data==',')  {ComaPoint++;    CharPoint=0;VTG[ComaPoint][0]=0;return;}
                if(data=='*') {return;}
                VTG[ComaPoint][CharPoint++]=data;
                VTG[ComaPoint][CharPoint]=0;
                return;
        case    0x4754564e:             //NVTG
                if(data==',')  {ComaPoint++;    CharPoint=0;VTG[ComaPoint][0]=0;return;}
                if(data=='*') {return;}
                VTG[ComaPoint][CharPoint++]=data;
                VTG[ComaPoint][CharPoint]=0;
                return;
        case    0x56534750:             //PGSV          
                if(data==',')  {ComaPoint++;    CharPoint=0;GSV[ComaPoint][0]=0;return;}
                if(data=='*')  {GPS_COUNT=atoi(ViewSat);MsgType=0;return;}
                GSV[ComaPoint][CharPoint++]=data;
                GSV[ComaPoint][CharPoint]=0;
                return;
        case    0x5653474c:             //LGSV          
                if(data==',')  {ComaPoint++;    CharPoint=0;GSV[ComaPoint][0]=0;return;}
                if(data=='*') {GLONAS_COUNT=atoi(ViewSat);MsgType=0;return;}
                GSV[ComaPoint][CharPoint++]=data;
                GSV[ComaPoint][CharPoint]=0;
                return;
        default:        ByteCount=0xff;break;
        }       
ByteCount=0xff;
}


void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		Parser(USART_ReceiveData(USART2));
	}
}


void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

		buf[i] = USART_ReceiveData(USART1);
		if (buf[i] == 0x81)
			{
				i = 0;
				buf[i] = 0x81;
			}
		i++;
		if((i == 7) && (buf[0] == 0x81) && (buf[5] == 0x2A)&&(buf[6]==0x2A))
			{
				Recieve_buf[2] = buf[2];
				Recieve_buf[3] = buf[3];	
				Recieve_buf[1] = buf[1];
				i = 0;
			}
		else 
			{ 
				if (i>7)i = 0;
			}
	}
}


void TIM3_IRQHandler(void)
{
if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		step_Y2++;

		if(Recieve_buf[1] == 0x2)
			{
				motor_Y2 = Recieve_buf[1];
				angle_Y2 = Recieve_buf[2];
				up_down_Y2 = Recieve_buf[3];
				Recieve_buf[1] = 0;
				Recieve_buf[2] = 0;
				Recieve_buf[3] = 0;
			}
		if(motor_Y2 == 2)
			{
				pulse_Y2 = (angle_Y2/0.9)*18;	
				if(!RUN_Y2)
				{
					tmppulse_Y2 = step_Y2 + pulse_Y2;
					flowing_start_Y2 = step_Y2 + 970;
					flowing_stop_Y2 = (step_Y2 + pulse_Y2) - 970;
				}
				RUN_Y2 = 1;
				if(up_down_Y2)GPIO_ResetBits(GPIOA , GPIO_Pin_8);	else GPIO_SetBits(GPIOA , GPIO_Pin_8);		
				PWM_Y2 = Period_Y2 / 2;
				if(pp_Y2)
					{
						Period_Y2 -=4;
						PWM_Y2 = Period_Y2 / 2;
						TIM3 -> ARR = Period_Y2;
						TIM3 -> CCR2 = PWM_Y2;
					}
				if(step_Y2 == flowing_start_Y2) pp_Y2 =0;
				TIM3->CCER |= TIM_CCER_CC2E;
				if(step_Y2 == flowing_stop_Y2) ps_Y2 =1;
				if(ps_Y2)
					{
						Period_Y2 +=4;
						PWM_Y2 = Period_Y2 / 2;
						TIM3 -> ARR = Period_Y2;
						TIM3 -> CCR2 = PWM_Y2;
					}
				if(step_Y2 == tmppulse_Y2)
					{
						TIM3->CCER &= ~TIM_CCER_CC2E;
						RUN_Y2 = 0;
						motor_Y2 = 0;
						tmppulse_Y2 = 0;
					}
			}		
	}

}

void TIM16_IRQHandler(void)
{
if (TIM_GetITStatus(TIM16, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
		step_Y1++;
		
		if(Recieve_buf[1] == 0x1)
			{
				motor_Y1 = Recieve_buf[1];
				angle_Y1 = Recieve_buf[2];
				up_down_Y1 = Recieve_buf[3];
				Recieve_buf[1] = 0;
				Recieve_buf[2] = 0;
				Recieve_buf[3] = 0;
			}
			
		if(motor_Y1 == 1)
			{
				pulse_Y1 = (angle_Y1/0.9)*18;
				if(!RUN_Y1)
					{
					tmppulse_Y1 = step_Y1 + pulse_Y1;
					flowing_start_Y1 = step_Y1 + 970;
					flowing_stop_Y1 = (step_Y1 + pulse_Y1) - 970;
					}	
				RUN_Y1 = 1;
				if(up_down_Y1)GPIO_SetBits(GPIOB , GPIO_Pin_4);	else GPIO_ResetBits(GPIOB , GPIO_Pin_4);
				PWM_Y1 = Period_Y1 / 2;
				if(pp_Y1)
					{
						Period_Y1 -=4;
						PWM_Y1 = Period_Y1 / 2;
						TIM3 -> ARR = Period_Y1;
						TIM3 -> CCR1 = PWM_Y1;
					}
				if(step_Y1 == flowing_start_Y1) pp_Y1 =0;
				TIM16->CCER |= TIM_CCER_CC1E;
				if(step_Y1 == flowing_stop_Y1) ps_Y1 =1;
				if(ps_Y1)
					{
						Period_Y1 +=4;
						PWM_Y1 = Period_Y1 / 2;
						TIM16 -> ARR = Period_Y1;
						TIM16 -> CCR1 = PWM_Y1;
					}
				if(step_Y1 == tmppulse_Y1)
					{
						TIM16->CCER &= ~TIM_CCER_CC1E;
						RUN_Y1 = 0;
						motor_Y1 = 0;
						tmppulse_Y1 = 0;
					}
			}
	}
}
void TIM17_IRQHandler(void)
{
if (TIM_GetITStatus(TIM17, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM17, TIM_IT_Update);
		step_Z++;
		
		if(Recieve_buf[1] == 0x3)
			{
				motor_Z = Recieve_buf[1];
				angle_Z = Recieve_buf[2];
				up_down_Z = Recieve_buf[3];
				Recieve_buf[1] = 0;
				Recieve_buf[2] = 0;
				Recieve_buf[3] = 0;
			}
		
		if(motor_Z == 3)
			{
				pulse_Z = (angle_Z/0.9)*18;
				if(!RUN_Z)
					{
					tmppulse_Z = step_Z + pulse_Z;
					flowing_start_Z = step_Z + 950;
					flowing_stop_Z = (step_Z + pulse_Z) - 950;
					}
				RUN_Z = 1;
				if(up_down_Z)GPIO_SetBits(GPIOB , GPIO_Pin_10);	else GPIO_ResetBits(GPIOB , GPIO_Pin_10);
				if(pp_Z)
					{
						Period_Z -=4;
						PWM_Z = Period_Z / 2;
						TIM17 -> ARR = Period_Z;
						TIM17 -> CCR1 = PWM_Z;
					}
				if(step_Z == flowing_start_Z) pp_Z =0;
				TIM17->CCER |= TIM_CCER_CC1E;
				if(step_Z == flowing_stop_Z) ps_Z =1;
				if(ps_Z)
					{
						Period_Z +=4;
						PWM_Z = Period_Z / 2;
						TIM17 -> ARR = Period_Z;
						TIM17 -> CCR1 = PWM_Z;
					}
				if(step_Z == tmppulse_Z)
					{
						TIM17->CCER &= ~TIM_CCER_CC1E;
						RUN_Z = 0;
						motor_Z = 0;
						tmppulse_Z = 0;
					}
			}
	}
}

int main(void)
{	
	RCC_HSEConfig(RCC_HSE_OFF);
 	RCC_HSICmd(ENABLE);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

	
	PORT_Config();
	TIM3_Config();
	TIM17_Config();
//	Hall_sensor();



  while (1)
  {
	}
}
