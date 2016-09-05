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
uint8_t CMD_Trigger = 0;
uint8_t Alt_latest = 0;
int compare_enough_data = 0;
int number_byte_empty_into_Buf_USART2 = 0;

int lenght_of_data_IMU_GPS=0;
//uint8_t IMU[80];
uint8_t GGA[100];
uint8_t VTG[100];

uint8_t CMD_delay=0;
uint16_t G_delay_count=0;
uint8_t Buf_USART2[300],Buf_UART4[300],Buf_rx4[9],Buf1_rx4[9];
uint8_t Update_heso_Roll=0,Update_heso_Pitch=0,Update_heso_Yaw=0,Update_heso_Alt=0,Update_heso_Press=0;

uint8_t crc;
int position_VTG = 0;
int i=0;
uint8_t find_char_$ = 0 ;
uint16_t position_end_of_VTG = 0, position_end_of_GGA = 0,l2=0;
uint8_t flag_set_or_current_press;
uint8_t flag_press=0;
uint8_t state_alt=1,state_press=0;
void xacnhan(uint8_t *buffer);

uint8_t test_simulate1, test_simulate2, test_simulate3;
		char str[] = "12345.56";
		double d;
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
    UART4_Configuration(115200);
    USART2_Configuration(115200);
    DMA_UART4_Configuration(Buf_USART2,300);
    DMA_USART2_Config(Buf_USART2,300);
    DMA_UART4_RX(Buf_rx4,9);
// defaude dieu khien o che do ALT  
    state_press=0;
    state_alt=1;


    power ();
    
    MyTIM_PWM_Configuration();  
       
    SysTick_Config(SystemCoreClock/500);
    GPIO_SetBits(GPIOB,GPIO_Pin_12);
		


		sscanf(str, "%lf", &d);

    while(1)
    {
        if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8))
        {
						Call_Roll_PID(Roll_PID.SetPoint);   
						//Gent_Pwm_Pitch(-5);
						Call_Pitch_PID(Pitch_PID.SetPoint);
						Call_Yaw_PID(Yaw_PID.SetPoint);
						//Call_Alt_PID(Alt_PID.SetPoint);
						Gent_Pwm_Alt(15);
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
    if( CMD_delay ==1 )
    {   
        G_delay_count += 1;
    }
    if (G_delay_count == 1000)//reset buffer receive data from groud staion
    {
        G_delay_count =0;
        CMD_delay =0;
    //  DMA_UART4_RX(Buf_rx4,9);
        DMA_Cmd(DMA1_Stream2,DISABLE);
				DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
        DMA_SetCurrDataCounter(DMA1_Stream2,9);
        DMA_Cmd(DMA1_Stream2,ENABLE);
    }         
    if(compare_enough_data == number_byte_empty_into_Buf_USART2)
    {
        lenght_of_data_IMU_GPS = 300 - number_byte_empty_into_Buf_USART2;//y number byte in buffer DMA has received
        if (lenght_of_data_IMU_GPS > 120)//buffer size IMU/GPS < 290
        {
             //fastest way to restart DMA
          DMA_Cmd(DMA1_Stream5,DISABLE);
          DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
          DMA_SetCurrDataCounter(DMA1_Stream5,300);
					DMA_Cmd(DMA1_Stream5,ENABLE);
          
					//test simulate
					test_simulate1 = Buf_USART2[3];
					test_simulate2 = Buf_USART2[4];
					test_simulate3 = Buf_USART2[5];
					//test with STM studio => Buf_USART2[0] = 0x0A, Buf_USART2[1] = 0x24 ('$'), Buf_USART2[2], Buf_USART2[3],... roll, pitch,...
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
             Sampling_RPY(Buf_USART2,80);//get roll, pitch, yaw
                    //Sampling_GGA(&Buf_USART2[position_VTG -1] ,position_end_of_VTG - position_VTG + 2);//get lat, lon
                    //Sampling_GGA(GGA1 ,26);
                    //Sampling_VTG(&Buf_USART2[l+1],l1-l);//get speed
                    //save data to SD card
                    //imu ( DATA[0--> h-2]  BAO GOM \N VA \S\R)
                    if ( CMD_Trigger == 0)//if CMD_Trigger =1 ; receive data from ground station
                    {//if CMD_Trigger = 0 ; transmit data to ground station
                            
                            DMA_SetCurrDataCounter(DMA1_Stream4,lenght_of_data_IMU_GPS);
                            DMA_Cmd(DMA1_Stream4,ENABLE);   
                        }
                        else
                        {
                            CMD_Trigger=0;
                            xacnhan(&Buf_rx4[0]);
                            CMD_delay =1 ;
//                          DMA_Cmd(DMA1_Stream2,DISABLE);
//                          DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
//                          DMA_SetCurrDataCounter(DMA1_Stream2,9);
//                          DMA_Cmd(DMA1_Stream2,ENABLE);
                        }     
        }
    }
 }

//Ngat nhan dmauart4
void DMA1_Stream2_IRQHandler(void)
{
        /* Test on DMA Stream Transfer Complete interrupt */
        if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
        {
            DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
            CMD_Trigger = 1 ;
            //tat DMA Tx                        
//          them1=1;
                        //restart DMA_RX
//          DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
                
        }
    }
void xacnhan(uint8_t *buffer)
    {
        int i;
        switch (buffer[0])
        {
            case 1:
                
                if(CRC_Cal(0,&buffer[0],7)==buffer[8]) 
                {
                    Buf_UART4[0]=13;
                    Buf_UART4[1]=10;
                    Buf_UART4[2]=buffer[0];
                    Buf_UART4[3]=6;//ACK
                    Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
                    Buf_UART4[5]=13;
                    DMA_SetCurrDataCounter(DMA1_Stream4,6);
                    DMA_Cmd(DMA1_Stream4,ENABLE);
                    Roll_PID.Kp=buffer[1]*0.1;
                    Roll_PID.Ki=buffer[2]*0.1;
                    Roll_PID.Kd=buffer[3]*0.1;
                    if(buffer[4]!=0)
                    {
                    Roll_PID.SetPoint=buffer[4];
                    }
                    Update_heso_Roll=1;
                }
                else
                {
                    Buf_UART4[0]=13;
                    Buf_UART4[1]=10;
                    Buf_UART4[2]=buffer[0];
                    Buf_UART4[3]=21;
                    Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
                    Buf_UART4[5]=13;
                    DMA_SetCurrDataCounter(DMA1_Stream4,6);
                    DMA_Cmd(DMA1_Stream4,ENABLE);
                }
                break;
                
            case 2:
                    if(CRC_Cal(0,&buffer[0],7)==buffer[8]) 
                    {
                    //  them=1;
                        Buf_UART4[0]=13;
                        Buf_UART4[1]=10;
                        Buf_UART4[2]=buffer[0];
                        Buf_UART4[3]=6;//ACK
                        Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
                        Buf_UART4[5]=13;
                        DMA_SetCurrDataCounter(DMA1_Stream4,6);
                        DMA_Cmd(DMA1_Stream4,ENABLE);
                        Pitch_PID.Kp=buffer[1]*0.1;
                        Pitch_PID.Ki=buffer[2]*0.1;
                        Pitch_PID.Kd=buffer[3]*0.1;
                        if(buffer[4]!=0)
                        {
                        Pitch_PID.SetPoint=buffer[4];
                        }
                        Update_heso_Pitch=1;
                    }
                    else
                    {
                        Buf_UART4[0]=13;
                        Buf_UART4[1]=10;
                        Buf_UART4[2]=buffer[0];
                        Buf_UART4[3]=21;
                        Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
                        Buf_UART4[5]=13;
                        DMA_SetCurrDataCounter(DMA1_Stream4,6);
                        DMA_Cmd(DMA1_Stream4,ENABLE);
                    }
                    break;
                    
                    case 3:
                        if(CRC_Cal(0,&buffer[0],7)==buffer[8]) 
                        {
                            Buf_UART4[0]=13;
                            Buf_UART4[1]=10;
                            Buf_UART4[2]=buffer[0];
                            Buf_UART4[3]=6;//ACK
                            Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
                            Buf_UART4[5]=13;
                            DMA_SetCurrDataCounter(DMA1_Stream4,6);
                            DMA_Cmd(DMA1_Stream4,ENABLE);
                            Yaw_PID.Kp=buffer[1]*0.1;
                            Yaw_PID.Ki=buffer[2]*0.1;
                            Yaw_PID.Kd=buffer[3]*0.1;
                            if(buffer[4]!=0)
                            {
                            Yaw_PID.SetPoint=buffer[4];
                            }
                            Update_heso_Yaw=1;
                        }
                        else
                        {
                            Buf_UART4[0]=13;
                            Buf_UART4[1]=10;
                            Buf_UART4[2]=buffer[0];
                            Buf_UART4[3]=21;
                            Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
                            Buf_UART4[5]=13;
                            DMA_SetCurrDataCounter(DMA1_Stream4,6);
                            DMA_Cmd(DMA1_Stream4,ENABLE);
                        }
                        break;
                        
                    case 4:
                            if(CRC_Cal(0,&buffer[0],7)==buffer[8]) 
                            {
                            Buf_UART4[0]=13;
                            Buf_UART4[1]=10;
                            Buf_UART4[2]=buffer[0];
                            Buf_UART4[3]=6;//ACK
                            Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
                            Buf_UART4[5]=13;
                            DMA_SetCurrDataCounter(DMA1_Stream4,6);
                            DMA_Cmd(DMA1_Stream4,ENABLE);
                            Alt_PID.Kp=buffer[1];
                            Alt_PID.Ki=buffer[2];
                            Alt_PID.Kd=buffer[3];
                                //if(buffer[4]!=0)  Alt_PID.SetPoint=buffer[4];
                            Update_heso_Alt=1;
                            state_press=0;
                            state_alt=1;
                            }
                        else
                        {
                            Buf_UART4[0]=13;
                            Buf_UART4[1]=10;
                            Buf_UART4[2]=buffer[0];
                            Buf_UART4[3]=21;
                            Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
                            Buf_UART4[5]=13;
                            DMA_SetCurrDataCounter(DMA1_Stream4,6);
                            DMA_Cmd(DMA1_Stream4,ENABLE);
                        }
                        break;
                    case 5:
                            if(CRC_Cal(0,&buffer[0],7)==buffer[8]) 
                            {
                                    Buf_UART4[0]=13;
                                    Buf_UART4[1]=10;
                                    Buf_UART4[2]=buffer[0];
                                    Buf_UART4[3]=6;//ACK
                                    Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
                                    Buf_UART4[5]=13;
                                    DMA_SetCurrDataCounter(DMA1_Stream4,6);
                                    DMA_Cmd(DMA1_Stream4,ENABLE);
//                                  Press.Kp=buffer[1];
//                                  Press.Ki=buffer[2];
//                                  Press.Kd=buffer[3];
                                    Alt_PID.Kp=buffer[1];
                                    Alt_PID.Ki=buffer[2];
                                    Alt_PID.Kd=buffer[3];
//                                  Update_heso_Press=1; 
                                    Update_heso_Alt=1;  
                                        
                                    state_press=1;
                                    state_alt=0;
                            }
                            else
                            {
                                    Buf_UART4[0]=13;
                                    Buf_UART4[1]=10;
                                    Buf_UART4[2]=buffer[0];
                                    Buf_UART4[3]=21;
                                    Buf_UART4[4]=CRC_Cal(1,&Buf_UART4[0],3);//CRC
                                    Buf_UART4[5]=13;
                                    DMA_SetCurrDataCounter(DMA1_Stream4,6);
                                    DMA_Cmd(DMA1_Stream4,ENABLE);
                            }
                            break;
                        default:
                            for ( i =0 ; i<9 ;++i )
												    {
                                *(buffer+i)=0;
														}
                            DMA_Cmd(DMA1_Stream2,DISABLE);
                            DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
                            DMA_SetCurrDataCounter(DMA1_Stream2,9);
                            DMA_Cmd(DMA1_Stream2,ENABLE);                   
                }
    }       
        


/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
// /*.................................ghi 9 chuoi...........................................*/          
