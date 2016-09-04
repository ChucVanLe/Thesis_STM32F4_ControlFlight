// /**************************************************************************************/
// #include "stm32f4xx.h" //10/9/2014
// #include "crc.h"
// #include "diskio.h"
// #include "ff.h"
// #include "integer.h"
// #include "string.h"
// #include "powersd.h"
// #include "rccmain.h"
// #include "uart.h"
// #include "dma.h"
// #include "stdio.h"
// #include "math.h"
// #include "tinhgoc.h"
// #include "tinhgps.h"
// /**************************************************************************************/
// void Delay(void) 
// 		{
//     uint32_t m;
//     for(m=0; m<0xffffff; ++m) 
// 		{
//    
//     }
// 		}
// /**************************************************************************************/
// //bien cho ham sd card
// FRESULT st_Result;   /* Result code */  
// FATFS fatfs;  /* File system object */  
// FIL fil;   /* File object */  
// DIR dir;   /* Directory object */  
// FILINFO fno;  /* File information object */  
// UINT *bw;
// const TCHAR *FilePath = "0:/HTDKN2/STM32.TXT" ; // file path 
// /**************************************************************************************/
// int  flag_writedata_sd=0;
// uint16_t Send_Sd_lenght=0;//buffer dung de gi sd card
// /**************************************************************************************/
// int counter_interupt=0,flag_into_main=0;//co dem so lan ngat va co vao chuong trinh chinh
// uint8_t Buffer[300];//buffer nhan dma
// uint8_t Buffer1[300];//buffer luu data 
// uint8_t  buffer[300];//buffer truyen dma
// uint8_t buffer1[300];	
// uint8_t data[9]={1,2,3,4,'.',5,6,7,8};
// /**************************************************************************************/
// uint8_t x_IMU[5],y_IMU[5],z_IMU[5];
// int count_$ =0,GGA_$ =0,GGA_13=0,VTG_13=0,dem_phay=0;
// int lenght_data_receive=0,h_gps=0,h_imu=0;

// /**************************************************************************************/
// //bien dem cho ham tickcount
// uint64_t TickCnt;
// uint64_t TickCnt1;
// uint64_t count;
// /**************************************************************************************/
// int count1=0;//bien dem dung cho dma nhan.
// int count2=0;//bien dem dung cho dma nhan.
// int i=0;//bien dem toan cuc.
// //int i2=0;//bien dem cho du lieu kinh,vi do.
// uint8_t crc;//bien dung cho tinh crc
// /**************************************************************************************/
// //bien cho pid
// typedef struct
// {
// 	float docao;
// 	float kinhdo;
// 	float vido;
// 	float vantocthang;
//   struct GOC
// 	{
// 		float x;
// 		float y;
// 		float z;
// 	}goc;
// }PID;
// PID pid;
// /**************************************************************************************/
// unsigned char  Send_Sd_Flag=0;
// uint8_t 	Send_Sd_Buf[300];
// uint8_t 	Send_Sd_Buf1[8000];
// uint8_t 	Send_Sd_Buf2[300];
// uint8_t 	Send_Sd_Buf3[300];
// uint8_t 	Send_Sd_Buf4[300];
// uint8_t 	Send_Sd_Buf5[300];
// uint8_t 	Send_Sd_Buf6[300];
// uint8_t 	Send_Sd_Buf7[300];
// uint8_t 	Send_Sd_Buf8[300];
// int flag_imu=0,flag_gps=0;
// /**************************************************************************************/
// uint64_t GetTickCount(void)
// {
//     return TickCnt;
// 	
// }
// /**************************************************************************************/
// //ham tick count
// uint8_t CheckTick(uint64_t TickBase, uint64_t Time)
// {
//     uint64_t CurTick;
//     
//     CurTick = GetTickCount();
//     
//     if (CurTick > TickBase)
//     {
//         if (CurTick >= (TickBase + Time))
//         {
//             return 1;
//         }
//     }
//     else
//     {
//         if (CurTick >= ((uint64_t)(TickBase + Time)))
//         {
//             return 1;
//         }
//     }

//     return 0;
// }
// /**************************************************************************************/
// // void randomm(void)
// // 	{
// // 		x=30;
// // 		//x=rand()%(doduong-doam+1)+doam;
// // 		a=b=d=((1+(x+45)/90)*16000)/8;
// // 		c=3500;
// // 	}
// // 	void randomm2(void)
// // 	{
// // 		x=-30;
// // 		//x=rand()%(doduong-doam+1)+doam;
// // 		a=b=d=((1+(x+45)/90)*16000)/8;
// // 		c=2000;
// // 	}
// /**************************************************************************************/
// void GPIO_Configuration(void);
// int main(void)
// {

// 	RCC_Configuration();
// 	power();
// 	NVIC_Configuration();
// 	GPIO_Configuration();
// 	UART4_Configuration(115200);
// 	USART2_Configuration(460800);
// 	DMA_UART4_Configuration(buffer,300);
// 	DMA_USART2_Config(Buffer,300);
// 	f_mount(0, &fatfs);  
//   f_mkdir ("0:/HTDKN2");	
//  	SysTick_Config(SystemCoreClock/500);

// 	while(1)
// 	{
// 						 
// 		if(flag_into_main==1)
// 		{
// 			flag_into_main=0;
// 			if(lenght_data_receive>100) 
// 			{
// 				counter_interupt=0;
// 				h_gps=lenght_data_receive;
// 				for(i=0;i<h_gps;i++)
// 				 {
// 					 Buffer1[i]=Buffer[i];
// 				 }
// 				 pid.goc.x= tinhgoc_x(x_IMU,Buffer1);
// 				 pid.goc.y= tinhgoc_y(y_IMU,Buffer1);
// 				 pid.goc.z =tinhgoc_z(z_IMU,Buffer1);

// 				
// 				//danh dau cac k tu $ cua GGA
// 				for(i=0;i<h_gps;i++)
// 				{
// 					if(Buffer1[i]=='$')
// 						{
// 							count_$++;
// 							if(count_$==1)
// 								{
// 									GGA_$=i;															
// 								}
// 						}
// 					if(Buffer1[i]==',')
// 					{
// 						dem_phay++;
// 						pid.vido=gps_vido(Buffer1);
// 						pid.kinhdo=gps_kinhdo(Buffer1);
// 						pid.docao=gps_docao(Buffer1);	
// 					}
// 					if((Buffer1[i]==13)&&(count_$==1))
// 						{
// 								GGA_13=i;
// 						}
// 					if((Buffer1[i]==13)&&(count_$==2))
// 					{
// 						VTG_13=i;
// 					}
// 				 }
// 				dem_phay=0;
// 	 //return vido;
// 	
// 				 
// 				//imu(DATA[0--> h-2] BAO GOM \N VA \S\R)
// 				buffer[0]=10;
// 				//buffer[1]=crc;
// 				buffer[2]=32;
// 				buffer[3]=73;//I
// 				buffer[4]=32;
// 				for(i=1;i<GGA_$-3;i++)
// 				{
// 					buffer[i+4]=Buffer1[i];	
// 				}
// 				buffer[GGA_$+1]=13;   
// 				crc=tinhcrc(3,buffer,GGA_$);	
// 				buffer[1]=crc;
// 				//GGA ( DATA[h-1-->l] BAO GOM \N VA \R )
// 				crc=tinhcrc(GGA_$,Buffer1,GGA_13-1);
// 				buffer[GGA_$+2]=10;
// 				buffer[GGA_$+3]=crc;
// 				buffer[GGA_$+4]=32;
// 				for(i=GGA_$;i<GGA_13+1;i++)
// 					{
// 						buffer[i+5]=Buffer1[i];
// 					}
// 				//VTG ( DATA[l+1 --> x-1] bao gom \n va \r )
// 				crc=tinhcrc(GGA_13+2,Buffer1,h_gps-2);
// 				buffer[GGA_13+6]=10;
// 				buffer[GGA_13+7]=crc;
// 				buffer[GGA_13+8]=32;
// 				for(i=GGA_13+2;i<h_gps;i++)
// 					{
// 						buffer[i+7]=Buffer1[i];
// 					}
// 				count_$=0;
// 				DMA_SetCurrDataCounter(DMA1_Stream4,h_gps+7);
// 				DMA_Cmd(DMA1_Stream4,ENABLE);	
// 				for(i=0;i<h_gps;i++)
// 					{
// 						Send_Sd_Buf[i]=Buffer1[i];
// 					}
//  				Send_Sd_lenght =h_gps;
// //  				Send_Sd_Flag=1;
// 				
// 			
// 			
// 				}			
// 		}
// 	}
// }

// /**************************************************************************************/
// void SysTick_Handler(void)
//  {
// 	count1=count2;
// 	count2=DMA1_Stream5->NDTR;
// 	if(count1==count2)
// 	{
// 	  lenght_data_receive=300-count2;
// 		if (lenght_data_receive>10)
// 		 {
// 			//fastest way to restart DMA
// 			DMA_Cmd(DMA1_Stream5,DISABLE);
// 			//while( DMA_GetCmdStatus(DMA1_Stream5) );
// 			DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
// 			DMA_SetCurrDataCounter(DMA1_Stream5,300);
// 			DMA_Cmd(DMA1_Stream5,ENABLE);		
// 			flag_into_main=1;
// 		 }
// 	}
// }
// 		

// /**************************************************************************************/
// /**************************************************************************************/
