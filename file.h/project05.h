/**luu project cu
  ******************************************************************************
  * @file   project.h 
  * @version initial version 1.0.0
  * @date		 18/10/2014	
  * @brief	 This file contain all macro and function header C source to use in my Project : GCS and Altitude Holding for fixed wing aircraft
						 It is include :
										- CRC calculator
										- PID function
										- Peripheral Init Function
  ******************************************************************************
  */
#ifndef __project_H
#define __project_H
#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "diskio.h"
#include "ff.h"
#include "integer.h"
#include "string.h"
#include "powersd.h"
#include "stdio.h"
#include "math.h"
/**************************************Function*******************************************/
#define Max_Xung 30
/*****************************Function CRC calculator*********************************************/
		uint8_t CRC_Cal(uint8_t offset,uint8_t *kitu,int lenght);
/*****************************PID Function*********************************************/
#define SPI_SD                   SPI1
#define RCC_GPIO_CS              RCC_AHB1Periph_GPIOA
#define GPIO_CS                  GPIOA
#define GPIO_SPI_SD              GPIOA
#define GPIO_Pin_CS              GPIO_Pin_4
#define GPIO_Pin_SPI_SD_SCK      GPIO_Pin_5
#define GPIO_Pin_SPI_SD_MISO     GPIO_Pin_6
#define GPIO_Pin_SPI_SD_MOSI     GPIO_Pin_7
#define RCC_SPI_SD               RCC_APB2Periph_SPI1
#define CS_H()      GPIO_SetBits(GPIO_CS, GPIO_Pin_CS) 
 /* - for SPI1 and full-speed APB2: 168MHz/2 */
 #define SPI_BaudRatePrescaler_SPI_SD  SPI_BaudRatePrescaler_8
extern SPI_InitTypeDef  SPI_InitStructure;
extern GPIO_InitTypeDef GPIO_InitStructure;
extern uint8_t Update_heso_Roll,Update_heso_Pitch,Update_heso_Yaw,Update_heso_Alt;
void power (void);
typedef struct
{
  volatile float Kp;
	volatile float Ki;
	volatile float Kd;
	volatile float e[3];
  volatile float Pid_Result;
	volatile float Pid_Result_Temp;
	volatile float a0,a1,a2;
	volatile float SetPoint;
	volatile float Current;
	volatile uint8_t enable;

}PID_Index;
extern PID_Index Roll_PID;
extern PID_Index Pitch_PID;
extern PID_Index Yaw_PID;
extern PID_Index Alt_PID;
extern PID_Index Latitude;
extern PID_Index Longitude ;
extern PID_Index Speed;
void PID_Init(void);
void Sampling_GGA(uint8_t* data,int lenght);
void Sampling_RPY(uint8_t * IMU , int lenght);
void Sampling_VTG(uint8_t* VTG , int lenght);
void Call_Roll_PID(float Roll_set);
void Call_Pitch_PID(float Pitch_set);
void Call_Yaw_PID(float Yaw_set);
void Call_Alt_PID(float Alt_set);
void Gent_Pwm_Roll(float Roll);
void Gent_Pwm_Pitch(float Pitch);
void Gent_Pwm_Yaw(float Yaw);
void Gent_Pwm_Alt(float Alt);
/*****************************Peripheral Function*********************************************/
float trituyetdoi(float a);
extern uint8_t Buf_USART2[],Buf_UART4[],IMU[],GGA[],VTG[],Buf_rx4[],Buf1_rx4[];
extern  USART_InitTypeDef USART_InitStructure;
extern uint64_t TickCnt;
extern DMA_InitTypeDef  DMA_InitStructure;
extern NVIC_InitTypeDef NVIC_InitStructure;
void MyRCC_Configuration(void);
void MyGPIO_Configuration(void);
void UART4_Configuration(uint32_t baudrate);
void USART2_Configuration (uint32_t baudrate);
void DMA_USART2_Config(uint8_t *buffer, uint16_t size) ;
void DMA_UART4_Configuration(uint8_t *buffer, uint16_t size);
void DMA1_Stream4_IRQHandler(void);
void NVIC_Configuration(void);
void DMA_UART4_RX(uint8_t *buffer, uint16_t size);
void Interrupt_uart4_rx(void);
void MyTIM_PWM_Configuration(void);
void EXTI_FPGA_Pa8(void);
void Delay_100ms(void);
/*****************************TickCount  *********************************************/


#endif
