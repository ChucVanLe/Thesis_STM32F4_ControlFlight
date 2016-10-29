/******************************************************************************
 *
 * thesis of LeVanChuc
 *
 ******************************************************************************/

/******************************************************************************
 *
 * Main controller of system
 *
 ******************************************************************************/

/******************************************************************************
 *
 * 	Module       : luanvan3-05vs2.c
 * 	Description  : This is code for STM32F405 to receive data from IMU/GPS, receive command from ground station
 *									process data, control flight in auto mode across FPGA
 *
 *  Tool           : keilC 4
 *  Chip           : STM32F405
 *  History        : 27-08-2016
 *  Version        : 1.0
 *
 *  Author         : Le Van Chuc, CLB NCKH DDT (levanchuc94qn@gmail.com)
 *  Notes          :
 *
******************************************************************************/
#include "project.h"
/**************************************************************************************/

/**************************************************************************************/
//bien cho ham sd card

/**************************************************************************************/
bool CMD_Trigger = false;//mode tran data to ground station or receive data from GS,CMD_Trigger = true: receive data from GS
uint8_t Alt_latest = 0;
int compare_enough_data = 0;
int number_byte_empty_into_Buf_USART2 = 0;
uint8_t number_byte_empty_into_Buf_USART4_rx = 0, index_find_e = 0;//variable for receive from GS
int lenght_of_data_IMU_GPS = 0;

uint8_t CMD_delay =0;
uint16_t G_delay_count =0;
uint8_t Buf_UART4[6],Buf_rx4[1];
char Buf_USART2[250], data_from_pc[250];
uint8_t Update_heso_Roll=0,Update_heso_Pitch=0,Update_heso_Yaw=0,Update_heso_Alt=0,Update_heso_Press=0;
#define		BUFF_SIZE			1//interrupt UART_DMA when receive BUFF_SIZE from GS
uint8_t index_receive_enough_data_GS = 0;//counter number of char from GS
uint8_t crc;
int i = 0;
uint8_t find_char_$ = 0 ;
uint16_t position_VTG = 0, position_end_of_VTG = 0, position_end_of_GGA = 0;
uint8_t flag_set_or_current_press;
uint8_t flag_press = 0;
uint8_t state_alt = 1,state_press = 0;
void receive_data_and_reply(char *buffer);

float lat_long_from_GS[14][2];//contain lat, long which is uploaded from GS
float test_simulate1, test_simulate2, test_simulate3;
/*************************************************************************************/
int main(void)
{
    Delay_100ms();
		MyRCC_Configuration();
    Delay_100ms();
    NVIC_Configuration();   
    Interrupt_uart4_rx();
    PID_Init();
		MyGPIO_Configuration();
    EXTI_FPGA_Pa8();
    UART4_Configuration(57600);
    USART2_Configuration(57600);
    DMA_UART4_Configuration((uint8_t*)Buf_USART2,250);
    DMA_USART2_Config((uint8_t*)Buf_USART2,250);
    DMA_UART4_RX(Buf_rx4, 1);
// defaude dieu khien o che do ALT  
    state_press = 0;
    state_alt = 1;

    //power();
    
    MyTIM_PWM_Configuration();  
       
    SysTick_Config(SystemCoreClock/500);//interrupt system 2ms
    GPIO_SetBits(GPIOB,GPIO_Pin_12);
		
    while(1)
    {
        if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8))
        {
						Call_Roll_PID(Roll_PID.SetPoint);   

						Call_Pitch_PID(Pitch_PID.SetPoint);
						Call_Yaw_PID(Yaw_PID.SetPoint);
						//test simute altitude
						if(Yaw_PID.Current > 0)
						Alt_PID.Current = Yaw_PID.Current;
						else Alt_PID.Current = 0;
						//Call_Alt_PID(Alt_PID.SetPoint);
						Call_Alt_PID(30);

					
//						Call_Roll_PID(15);   
//						Call_Pitch_PID(15);
//						Call_Yaw_PID(15);
//						//Call_Alt_PID(Alt_PID.SetPoint);
//						Gent_Pwm_Alt(15);
					
        }      
// /*.................................save data to SD Card...........................................*/  
//...........................					remove code save data because Mr.Huan has done it.				        
    }// end while
}//end main
    
/**************************************************************************************/
void SysTick_Handler(void)
 {
     
    compare_enough_data = number_byte_empty_into_Buf_USART2;
    number_byte_empty_into_Buf_USART2 = DMA1_Stream5->NDTR;// (Buffer->size) --> count2, number byte data from IMU/GPS
    if( CMD_delay == 1 )
    {   
        G_delay_count += 1;
    }


		if(CMD_Trigger) //receive enough 1 frame
		{//if CMD_Trigger = 1 ; receive data from ground station 
				receive_data_and_reply(&data_from_pc[0]);
				
				//reset data buffer
				CMD_Trigger = false;
				for(index_find_e = 0; index_find_e < 250; index_find_e ++)
						data_from_pc[index_find_e] = 0;
		}
		else
    if(compare_enough_data == number_byte_empty_into_Buf_USART2)
    {
        lenght_of_data_IMU_GPS = 250 - number_byte_empty_into_Buf_USART2;//y number byte in buffer DMA has received
        if (lenght_of_data_IMU_GPS > 100)//buffer size IMU/GPS < 290
        {
             //fastest way to restart DMA
          DMA_Cmd(DMA1_Stream5,DISABLE);
          DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
          DMA_SetCurrDataCounter(DMA1_Stream5,250);
					DMA_Cmd(DMA1_Stream5,ENABLE);
          

					//test with STM studio => Buf_USART2[0] = 0x0A, Buf_USART2[1] = 0x24 ('$'), Buf_USART2[2], Buf_USART2[3],... roll, pitch,... finish with '\r'
          /*
					$  0015  0068 -0080  0011 -0082  0123 -0203 -0019  0976  0297  0097  0383  0792 
					$GPVTG,350.26,T,,M,1.034,N,1.916,K,A*36
					$GPGGA,144845.10,1045.65955,N,10639.71225,E,1,05,11.37,89.0,M,-2.5,M,,*4B
					*/
					for(i = 0;i<lenght_of_data_IMU_GPS;i++)
					{
						if(Buf_USART2[i] == '$')
							{
								//if i1 == 0;
								//$ dau tien la chuoi  GGA, vi tri $ la h, vi tri ket thuc la l
								//$ thu 2 la chuoi VTG,vi tri ket thuc la l1
								find_char_$ ++;         
                if(find_char_$ == 2)
										position_VTG = i;//luu vi tri VTG_$                                                          										
               }
						if((Buf_USART2[i]==13)&&(find_char_$ == 2))
               {
									position_end_of_VTG = i;//luu vi tri VTG_13
               }
            if((Buf_USART2[i]==13)&&(find_char_$ == 3))
               { 
                  position_end_of_GGA = i;//luu vi tri GGA_13
                  find_char_$ = 0;
                  break;
               }
           }               
					Sampling_RPY((uint8_t*)Buf_USART2,80);//get roll, pitch, yaw
          Sampling_VTG(&Buf_USART2[position_VTG] ,position_end_of_VTG - position_VTG);//get speed
					Sampling_GGA(&Buf_USART2[position_end_of_VTG + 1], position_end_of_GGA - position_end_of_VTG);//get lat, lon, alt
          

//if (CMD_Trigger == 0)//if CMD_Trigger = 0 ; transmit data to ground station             
          DMA_SetCurrDataCounter(DMA1_Stream4,lenght_of_data_IMU_GPS);
          DMA_Cmd(DMA1_Stream4,ENABLE);   
 
        }
    }
 }

//Ngat nhan dmauart4
void DMA1_Stream2_IRQHandler(void)
{

		/* Clear the DMA1_Stream2 TCIF2 pending bit */
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);//DMA_IT_TCIF2 //co de bao day chua
		//format !data@
		if('!' == Buf_rx4[0])
			index_receive_enough_data_GS = 0;
		else index_receive_enough_data_GS++;
		
		data_from_pc[index_receive_enough_data_GS] = Buf_rx4[0];//copy du lieu ra, DMA_Mode_Circular: con tro tu quanh lai
		//mode nomal thi phai nop lai con tro
		//muon chay lan 2 thi enable lai
		//moi stream co 1 cai co, xoa co do thi moi bay DMA len duoc
		if('@' == Buf_rx4[0])//nhan duoc ky tu ket thuc chuoi bat dau xu ly
		{
			CMD_Trigger = true;
		}	
		/* reload new buff size for next reception */
		DMA1_Stream2->NDTR = BUFF_SIZE;
		DMA_Cmd(DMA1_Stream2, ENABLE);
}
void receive_data_and_reply(char *buffer)
{
	uint8_t i = 0, index_array_lat_lon = 0;
	uint8_t comma = 0;//find ','
	uint8_t fp_kp = 0, fp_ki = 0, fp_kd = 0, fp_setpoint = 0, ep_setpoint = 0, fp_lat = 0, fp_lon = 0;
	char temp_kp[10], temp_ki[10],temp_kd[10],temp_checksum[10],temp_setpoint[10], temp_lat[14], temp_long[14];
	uint32_t checksum_from_GS = 0, cal_check_sum = 0;
	//reset temp variable 
	for(i = 0; i < 10; i++)
	{
		temp_kp[i] = 0;
		temp_ki[i] = 0;
		temp_kd[i] = 0;
		temp_setpoint[i] = 0;
	}
	for(i = 0; i < 14; i++)
	{
		temp_lat[i] = 0;
		temp_long[i] = 0;
		lat_long_from_GS[i][0] = 0;
		lat_long_from_GS[i][1] = 0;
	}
	
	if('4' != buffer[1])//'4' = buffer[1]: upload latitude, longtitude
	{
		for (i=0; i <= index_receive_enough_data_GS; i++)
		{
			if (*(buffer + i)==',')
			{
				comma++;
				if(comma==1) fp_kp = i;
				if(comma==2) fp_ki = i;
				if(comma==3) fp_kd = i;
				if(comma==4) fp_setpoint = i;
				if(comma==5) ep_setpoint = i;
			}
		}
				//check sum
		for (i=1; i <= ep_setpoint; i++)
			{
				 cal_check_sum += *(buffer + i);
			}
	}

  switch (buffer[1])
				//'0': update roll
				//'1': update pitch
				//'2': update yaw
				//'3': update alt, state_alt = 1
				//'4': update lat, long
				//	update alt, state_alt = 0, no GPS, altitude is calculator with press, state press = 1
  {
  case '0':
	{

		//check error
		strncpy(temp_checksum, &buffer[ep_setpoint + 1], index_receive_enough_data_GS - ep_setpoint - 1);
		checksum_from_GS = atof(temp_checksum);
		if(checksum_from_GS == cal_check_sum)//send ACK to GS
		{
			strncpy(temp_kp, &buffer[fp_kp + 1], fp_ki - fp_kp - 1);
			Roll_PID.Kp = atof(temp_kp);
			strncpy(temp_ki, &buffer[fp_ki + 1], fp_kd - fp_ki - 1);
			Roll_PID.Ki = atof(temp_ki);
			strncpy(temp_kd, &buffer[fp_kd + 1], fp_setpoint - fp_kd - 1);
			Roll_PID.Kd = atof(temp_kd);
			strncpy(temp_setpoint, &buffer[fp_setpoint + 1], ep_setpoint - fp_setpoint - 1);
			Roll_PID.SetPoint = atof(temp_setpoint);
			Update_heso_Roll=1;
			//send reply to GS
      Buf_USART2[0] = '!';
      Buf_USART2[1] = buffer[1];
      Buf_USART2[2] = 6;//ACK
      Buf_USART2[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		else
		{
			Buf_USART2[0] = '!';
      Buf_USART2[1] = buffer[1];
      Buf_USART2[2] = 21;//NAK
      Buf_USART2[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}

    break;
	}
	case '1':
	{

    //check error
		strncpy(temp_checksum, &buffer[ep_setpoint + 1], index_receive_enough_data_GS - ep_setpoint - 1);
		checksum_from_GS = atof(temp_checksum);
		if(checksum_from_GS == cal_check_sum)//send ACK to GS
		{
			strncpy(temp_kp, &buffer[fp_kp + 1], fp_ki - fp_kp - 1);
			Pitch_PID.Kp = atof(temp_kp);
			strncpy(temp_ki, &buffer[fp_ki + 1], fp_kd - fp_ki - 1);
			Pitch_PID.Ki = atof(temp_ki);
			strncpy(temp_kd, &buffer[fp_kd + 1], fp_setpoint - fp_kd - 1);
			Pitch_PID.Kd = atof(temp_kd);
			strncpy(temp_setpoint, &buffer[fp_setpoint + 1], ep_setpoint - fp_setpoint - 1);
			Pitch_PID.SetPoint = atof(temp_setpoint);
			Update_heso_Pitch=1;
			//send reply to GS
      Buf_USART2[0] = '!';
      Buf_USART2[1] = buffer[1];
      Buf_USART2[2] = 6;//ACK
      Buf_USART2[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		else
		{
			Buf_USART2[0] = '!';
      Buf_USART2[1] = buffer[1];
      Buf_USART2[2] = 21;//NAK
      Buf_USART2[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		break;
	}
	case '2':
	{

    //check error
		strncpy(temp_checksum, &buffer[ep_setpoint + 1], index_receive_enough_data_GS - ep_setpoint - 1);
		checksum_from_GS = atof(temp_checksum);
		if(checksum_from_GS == cal_check_sum)//send ACK to GS
		{
			strncpy(temp_kp, &buffer[fp_kp + 1], fp_ki - fp_kp - 1);
			Yaw_PID.Kp = atof(temp_kp);
			strncpy(temp_ki, &buffer[fp_ki + 1], fp_kd - fp_ki - 1);
			Yaw_PID.Ki = atof(temp_ki);
			strncpy(temp_kd, &buffer[fp_kd + 1], fp_setpoint - fp_kd - 1);
			Yaw_PID.Kd = atof(temp_kd);
			strncpy(temp_setpoint, &buffer[fp_setpoint + 1], ep_setpoint - fp_setpoint - 1);
			Yaw_PID.SetPoint = atof(temp_setpoint);
			Update_heso_Yaw=1;
			//send reply to GS
      Buf_USART2[0] = '!';
      Buf_USART2[1] = buffer[1];
      Buf_USART2[2] = 6;//ACK
      Buf_USART2[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		else
		{
			Buf_USART2[0] = '!';
      Buf_USART2[1] = buffer[1];
      Buf_USART2[2] = 21;//NAK
      Buf_USART2[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		break;
	}
	case '3':
	{

    //check error
		strncpy(temp_checksum, &buffer[ep_setpoint + 1], index_receive_enough_data_GS - ep_setpoint - 1);
		checksum_from_GS = atof(temp_checksum);
		if(checksum_from_GS == cal_check_sum)//send ACK to GS
		{
			strncpy(temp_kp, &buffer[fp_kp + 1], fp_ki - fp_kp - 1);
			Alt_PID.Kp = atof(temp_kp);
			strncpy(temp_ki, &buffer[fp_ki + 1], fp_kd - fp_ki - 1);
			Alt_PID.Ki = atof(temp_ki);
			strncpy(temp_kd, &buffer[fp_kd + 1], fp_setpoint - fp_kd - 1);
			Alt_PID.Kd = atof(temp_kd);
			strncpy(temp_setpoint, &buffer[fp_setpoint + 1], ep_setpoint - fp_setpoint - 1);
			Alt_PID.SetPoint = atof(temp_setpoint);
			Update_heso_Alt=1;
			//send reply to GS
      Buf_USART2[0] = '!';
      Buf_USART2[1] = buffer[1];
      Buf_USART2[2] = 6;//ACK
      Buf_USART2[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		else
		{
			Buf_USART2[0] = '!';
      Buf_USART2[1] = buffer[1];
      Buf_USART2[2] = 21;//NAK
      Buf_USART2[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		break;
	}
	case '4'://update lat, long
	{//! 4 v lat k lon v lat k lon v lat k lon checksum @
		for (i=0; i <= index_receive_enough_data_GS; i++)
		{
			if(',' == *(buffer + i))//check sum
			{
				ep_setpoint = i;
				test_simulate1 = 1;
			}
		}		
		for (i=1; i <= ep_setpoint; i++)
		{
			 cal_check_sum += *(buffer + i);
		}
    //check error
		strncpy(temp_checksum, &buffer[ep_setpoint + 1], index_receive_enough_data_GS - ep_setpoint - 1);
		checksum_from_GS = atof(temp_checksum);
		if(checksum_from_GS == cal_check_sum)//send ACK to GS
		{
			//! 4 v lat k lon v lat k lon v lat k lon checksum @
			for (i=0; i <= index_receive_enough_data_GS; i++)
			{
				if ('k' == *(buffer + i)) 
				{
					fp_lon = i;			
					strncpy(temp_lat, &buffer[fp_lat + 1], fp_lon - fp_lat - 1);
					lat_long_from_GS[index_array_lat_lon][0] = atof(temp_lat);
				}
				else
				{
					if ('v' == *(buffer + i)) 
					{
						fp_lat = i;
						if(2 != fp_lat)
						{
						strncpy(temp_long, &buffer[fp_lon + 1], fp_lat - fp_lon - 1);
						lat_long_from_GS[index_array_lat_lon++][1] = atof(temp_long);
						}
					}
				}
			}
			//send respond to GS
      Buf_USART2[0] = '!';
      Buf_USART2[1] = buffer[1];
      Buf_USART2[2] = 6;//ACK
      Buf_USART2[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}
		else
		{
			Buf_USART2[0] = '!';
      Buf_USART2[1] = buffer[1];
      Buf_USART2[2] = 21;//NAK
      Buf_USART2[3] = '@';
			DMA_SetCurrDataCounter(DMA1_Stream4,4);
      DMA_Cmd(DMA1_Stream4,ENABLE);
		}			

			break;
	}
                

//                        default:
//                            for ( i =0 ; i<9 ;++i )
//												    {
//                                *(buffer+i)=0;
//														}
//                            DMA_Cmd(DMA1_Stream2,DISABLE);
//                            DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
//                            DMA_SetCurrDataCounter(DMA1_Stream2,9);
//                            DMA_Cmd(DMA1_Stream2,ENABLE);                   
//                }
			}
}       
        


/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
// /*.................................ghi 9 chuoi...........................................*/          
