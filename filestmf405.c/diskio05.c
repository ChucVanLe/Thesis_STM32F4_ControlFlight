/*-------------------------------------------------------------*/
#include <stm32f4xx.h>
#include "diskio.h"
#include "powersd.h"
/*-----------------------------------------------------*/
/* Hardware Configuration															 */
/*-----------------------------------------------------*/

 #define SPI_SD                   SPI1//
 #define RCC_GPIO_CS              RCC_AHB1Periph_GPIOA//
 #define GPIO_CS                  GPIOA//
 #define GPIO_SPI_SD              GPIOA//
 #define GPIO_Pin_CS              GPIO_Pin_4//
 #define GPIO_Pin_SPI_SD_SCK      GPIO_Pin_5//
 #define GPIO_Pin_SPI_SD_MISO     GPIO_Pin_6//
 #define GPIO_Pin_SPI_SD_MOSI     GPIO_Pin_7//
 #define RCC_SPI_SD               RCC_APB2Periph_SPI1//
 /* - for SPI1 and full-speed APB2: 168MHz/2 */
 #define SPI_BaudRatePrescaler_SPI_SD  SPI_BaudRatePrescaler_8 //

/* Definitions for MMC/SDC command */
#define CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define CMD1	(0x40+1)	/* SEND_OP_COND (MMC) */
#define ACMD41	(0xC0+41)	/* SEND_OP_COND (SDC) */
#define CMD8 	(0x40+8)	/* SEND_IF_COND */
#define CMD9 	(0x40+9)	/* SEND_CSD */
#define CMD10	(0x40+10)	/* SEND_CID */
#define CMD12	(0x40+12)	/* STOP_TRANSMISSION */
#define CMD13	(0x40+13)	/* SEND_STATUS */
#define ACMD13	(0xC0+13)	/* SD_STATUS (SDC) */
#define CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD18	(0x40+18)	/* READ_MULTIPLE_BLOCK */
#define CMD23	(0x40+23)	/* SET_BLOCK_COUNT (MMC) */
#define ACMD23	(0xC0+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(0x40+24)	/* WRITE_BLOCK */
#define CMD25	(0x40+25)	/* WRITE_MULTIPLE_BLOCK */
#define CMD55	(0x40+55)	/* APP_CMD */
#define CMD58	(0x40+58)	/* READ_OCR */

#define CS_L()        GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS)    /* MMC CS = L */	//
#define CS_H()      GPIO_SetBits(GPIO_CS, GPIO_Pin_CS)      /* MMC CS = H */		//

/* Card type flags (CardType) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
//#define CT_SDC		0x06		/* SD */
#define CT_BLOCK	0x08		/* Block addressing */

#if (_MAX_SS != 512) || (_FS_READONLY == 0) || (STM32_SD_DISK_IOCTRL_FORCE == 1)
#define STM32_SD_DISK_IOCTRL   1
#else
#define STM32_SD_DISK_IOCTRL   0
#endif

static
DSTATUS Stat = STA_NOINIT;	/* Disk status */

static
BYTE CardType;			/* b0:MMC, b1:SDv1, b2:SDv2, b3:Block addressing */


/*--------------------------------------------------------------------------

   Module Private Functions and Variables

---------------------------------------------------------------------------*/
enum speed_setting { INTERFACE_SLOW, INTERFACE_FAST };

static void interface_speed( enum speed_setting speed)
{
	uint32_t tmp;

	tmp = SPI_SD->CR1;
	if (speed== INTERFACE_SLOW ) {
		/* Set slow clock (100k-400k) */
		tmp = ( tmp | SPI_BaudRatePrescaler_256);
	} else {
		/* Set fast clock (depends on the CSD) */
		tmp = ( tmp & ~SPI_BaudRatePrescaler_256) | SPI_BaudRatePrescaler_SPI_SD;
	}
	SPI_SD->CR1 = tmp;
}


/*-----------------------------------------------------------------------*/
/* Transmit/Receive a byte to MMC via SPI  (Platform dependent)          */
/*-----------------------------------------------------------------------*/
static uint8_t stm32_spi_rw(  uint8_t out )
{
	/* Loop while DR register in not empty */
	// while (SPI_I2S_GetFlagStatus(SPI_SD, SPI_I2S_FLAG_TXE) == RESET) { ; }

	/* Send byte through the SPI peripheral */
	SPI_I2S_SendData(SPI_SD, out);
	/* Loop while DR register is empty */
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI_SD, SPI_I2S_FLAG_RXNE) == RESET) { ; }

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI_SD);
}



/*-----------------------------------------------------------------------*/
/* Transmit a byte to MMC via SPI  (Platform dependent)                  */
/*-----------------------------------------------------------------------*/

#define xmit_spi(dat)  stm32_spi_rw(dat)

/*-----------------------------------------------------------------------*/
/* Receive a byte from MMC via SPI  (Platform dependent)                 */
/*-----------------------------------------------------------------------*/
/* Receive a response when sent oxff*/
static uint8_t rcvr_spi (void)
{
	return stm32_spi_rw(0xff);
}

/* Alternative macro to receive data fast */
/* dest is address of the memory where held response*/
#define rcvr_spi_m(dst)  *(dst)=stm32_spi_rw(0xff)

/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static int wait_ready (void)
{
	BYTE res;
	UINT i;
	i = 82000;
	
	rcvr_spi(); /* Tranmit oxff and receive response by stm32_spi_rw (oxff) */
	do{
		res = rcvr_spi();
		i-- ;
	}
	while ((res != 0xFF) && i>0);
	return i? 1:0;
}

/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/

static void deselect (void)
{
	CS_H();
	rcvr_spi();	/* Dummy clock (force DO hi-z for multiple slave SPI) */
}

/*-----------------------------------------------------------------------*/
/* Select the card and wait for ready                                    */
/*-----------------------------------------------------------------------*/

static int select (void)	/* 1:OK, 0:Timeout */
{
	CS_L();
	rcvr_spi();	/* Dummy clock (force DO enabled) */

	if (wait_ready()) return 1;	/* OK */
	deselect();
	return 0;			/* Failed */
}

/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/

static
void release_spi (void)
{
	deselect(); /* CS= HIGHT*/
	rcvr_spi();   /* Need 8 cycle clocks to  succesively finish*/
}

/*-----------------------------------------------------------------------*/
/* Power Control and interface-initialization (Platform dependent)       */
/*-----------------------------------------------------------------------*/
#ifdef STM32_SD_USE_DMA
/*-----------------------------------------------------------------------*/
/* Transmit/Receive Block using DMA (Platform dependent. STM32 here)     */
/*-----------------------------------------------------------------------*/
static
void stm32_dma_transfer(
	BOOL receive,		/* FALSE for buff->SPI, TRUE for SPI->buff               */
	const BYTE *buff,	/* receive TRUE  : 512 byte data block to be transmitted
						   receive FALSE : Data buffer to store received data    */
	UINT btr 			/* receive TRUE  : Byte count (must be multiple of 2)
						   receive FALSE : Byte count (must be 512)              */
)
{
	DMA_InitTypeDef DMA_InitStructure;
	WORD rw_workbyte[] = { 0xffff };
	DMA_DeInit(DMA2_Stream0);//SPI-RX
	DMA_DeInit(DMA2_Stream5);//SPI-TX
	//RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_DMA2, ENABLE); 
	DMA_InitStructure.DMA_Channel        = DMA_Channel_3;
	/* shared DMA configuration values */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (DWORD)(&(SPI_SD->DR));
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_BufferSize = btr;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

// 	DMA_DeInit(DMA2_Stream0);//SPI-RX
// 	DMA_DeInit(DMA2_Stream5);//SPI-TX

	if ( receive ) {

		/* DMA2 channel3 configuration SPI1 RX ---------------------------------------------*/
		/* DMA2 channel3 configuration SPI2 RX ---------------------------------------------*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (DWORD)buff;
		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_Init(DMA2_Stream0, &DMA_InitStructure);

		/* DMA1 channel3 configuration SPI1 TX ---------------------------------------------*/
		/* DMA1 channel5 configuration SPI2 TX ---------------------------------------------*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (DWORD)rw_workbyte;
		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
		DMA_Init(DMA2_Stream5, &DMA_InitStructure);

	} else {

#if _FS_READONLY == 0
		/* DMA1 channel2 configuration SPI1 RX ---------------------------------------------*/
		/* DMA1 channel4 configuration SPI2 RX ---------------------------------------------*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (DWORD)rw_workbyte;
		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
		DMA_Init(DMA2_Stream0, &DMA_InitStructure);

		/* DMA1 channel3 configuration SPI1 TX ---------------------------------------------*/
		/* DMA1 channel5 configuration SPI2 TX ---------------------------------------------*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (DWORD)buff;
		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_Init(DMA2_Stream5, &DMA_InitStructure);
#endif

	}

	/* Enable DMA RX Channel */
	DMA_Cmd(DMA2_Stream0, ENABLE);
	/* Enable DMA TX Channel */
	DMA_Cmd(DMA2_Stream5, ENABLE);

	/* Enable SPI TX/RX request */
	SPI_I2S_DMACmd(SPI_SD, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	/* Wait until DMA1_Channel 3 Transfer Complete */
	/// not needed: while (DMA_GetFlagStatus(DMA_FLAG_SPI_SD_TC_TX) == RESET) { ; }
	/* Wait until DMA1_Channel 2 Receive Complete */
	//while (DMA_GetFlagStatus(DMA_FLAG_SPI_SD_TC_RX) == RESET) { ; }
	// same w/o function-call:
	// while ( ( ( DMA1->ISR ) & DMA_FLAG_SPI_SD_TC_RX ) == RESET ) { ; }

	/* Disable DMA RX Channel */
	DMA_Cmd(DMA2_Stream0, DISABLE);
	/* Disable DMA TX Channel */
	DMA_Cmd(DMA2_Stream5, DISABLE);

	/* Disable SPI RX/TX request */
	SPI_I2S_DMACmd(SPI_SD, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);
	//DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TCIF5);
}
#endif /* STM32_SD_USE_DMA */


static
void power_off (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	select();
	wait_ready();/* escape if receive is oxff or time2 is out*/
	release_spi(); /* CS = HIGH*/

	SPI_I2S_DeInit(SPI_SD);
	SPI_Cmd(SPI_SD, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_SPI_SD, DISABLE);

	/* All SPI-Pins to input with weak internal pull-downs */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_SPI_SD_SCK | GPIO_Pin_SPI_SD_MISO | GPIO_Pin_SPI_SD_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIO_SPI_SD, &GPIO_InitStructure);
	Stat |= STA_NOINIT;
}


/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static int rcvr_datablock ( /* 1:OK, 0:Failed */
 	BYTE *buff,			/* Data buffer to store received data */
	UINT  btr			/* Byte count (must be multiple of 4) */
)
{
	BYTE token;
	UINT i;

	i=9000;
	do {							/* Wait for data packet in timeout of 100ms */
		token = rcvr_spi();		/* Wait for valid response oxfe*/
		i--;
	} while ((token == 0xFF) && i>0); /*if escape, token = oxff if timer1 is out, else token=oxfe*/
	if(token != 0xFE) return 0;	/* If not valid data token, return with error */
	
// 	do {							/* Receive the data block into buffer */
// 		rcvr_spi_m(buff++);
// 		rcvr_spi_m(buff++);
// 		rcvr_spi_m(buff++);
// 		rcvr_spi_m(buff++);
// 	} while (btr -= 4);
#ifdef STM32_SD_USE_DMA
stm32_dma_transfer( TRUE, buff, btr );
#else
do {							/* Receive the data block into buffer */
	rcvr_spi_m(buff++);
	rcvr_spi_m(buff++);
	rcvr_spi_m(buff++);
	rcvr_spi_m(buff++);
} while (btr -= 4);
#endif /* STM32_SD_USE_DMA */

	rcvr_spi();						/* Discard CRC */
	rcvr_spi();

	return 1;					/* Return with success */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/
/* No use in my thesic */
#if _FS_READONLY == 0
static int xmit_datablock ( /* 1:OK, 0:Failed */
	const BYTE *buff,	/* 512 byte data block to be transmitted */
	BYTE  token			/* Data/Stop token */
)
{
	BYTE resp;
	BYTE  wc;
	if (!wait_ready()) return 0;

	xmit_spi(token);					/* transmit data token */
	if (token != 0xFD) {		/* Is data token */

		wc = 0;
// 		do {							/* transmit the 512 byte data block to MMC */
// 			xmit_spi(*buff++);
// 			xmit_spi(*buff++);
// 		} while (--wc);
// #ifndef STM32_SD_USE_DMA
// 	BYTE wc;
// #endif

// 	if (wait_ready() != 0xFF) return 0;

// 	xmit_spi(token);					/* transmit data token */
// 	if (token != 0xFD) {	/* Is data token */

#ifdef STM32_SD_USE_DMA
		stm32_dma_transfer( FALSE, buff, 512 );
#else
		wc = 0;
		do {							/* transmit the 512 byte data block to MMC */
			xmit_spi(*buff++);
			xmit_spi(*buff++);
		} while (--wc);
#endif /* STM32_SD_USE_DMA */

		xmit_spi(0xFF);					/* CRC (Dummy) */
		xmit_spi(0xFF);
		resp = rcvr_spi();				/* Receive data response */
		if ((resp & 0x1F) != 0x05) {		/* If not accepted, return with error */
			return 0;
		}
	}						/* illegal command and in idle state*/
	return 1;
}
#endif /* _READONLY */


/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static BYTE send_cmd (
	BYTE cmd,		/* Command byte */
	DWORD arg		/* Argument  usually is address*/
)
{
	BYTE n, res;
	
	if (cmd & 0x80) {	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
		cmd &= 0x7F;
		n = send_cmd(CMD55, 0);
		if (n > 1) return n;
	}

	/* Select the card and wait for ready */
	deselect();
	if(!select()) return 0xFF;
	
	/* Send command packet */
	xmit_spi(cmd);						/* Start + Command index */
	xmit_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
	xmit_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
	xmit_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
	xmit_spi((BYTE)arg);				/* Argument[7..0] */
	n = 0x01;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
	xmit_spi(n);

	/* Receive command response */
	if (cmd == CMD12) rcvr_spi();		/* Skip a stuff byte when stop reading */
	
	n = 10;								/* Wait for a valid response in timeout of 10 attempts */
	do
		res = rcvr_spi();
	while ((res & 0x80) && --n); /* wait for valid respose*/

	return res;			/* Return with the response value */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE drv			/* Drive number (always 0) */
)
{
	DSTATUS s;


	if (drv) return STA_NOINIT;

	/* Check if the card is kept initialized */
	s = Stat;
	if (!(s & STA_NOINIT)) {
		if (send_cmd(CMD13, 0))	/* Read card status */
			s = STA_NOINIT;
		rcvr_spi();		/* Receive following half of R2 */
		deselect();
	}
	Stat = s;

	return s;
}

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE drv		/* Physical drive nmuber (0) */
)
{
	BYTE n, ty, cmd, buf[4];
	UINT tmr;
	DSTATUS s;


	if (drv) return RES_NOTRDY;

	interface_speed(INTERFACE_SLOW);
	
	for (n = 10; n; n--) rcvr_spi();	/* 80 dummy clocks */	

	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? */
			rcvr_spi();				/* Get trailing return value of R7 resp */
			rcvr_spi();
			buf[2] = rcvr_spi();
			buf[3] = rcvr_spi();
			if (buf[2] == 0x01 && buf[3] == 0xAA) {		/* The card can work at vdd range of 2.7-3.6V */
				for (tmr = 8200; tmr; tmr--) {			/* Wait for leaving idle state (ACMD41 with HCS bit) */
					if (send_cmd(ACMD41, 1UL << 30) == 0) break;
				}
				if (tmr && send_cmd(CMD58, 0) == 0) {	/* Check CCS bit in the OCR */
					//buf[0] = rcvr_spi();
/*.....................................................................................................................*/
					while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
					SPI1->DR = 0xFF;
					/* Wait for SPI2 data reception */
					while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
					buf[0] = SPI1->DR;
/*.....................................................................................................................*/
					rcvr_spi();
					rcvr_spi();
					rcvr_spi();
					ty = (buf[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 */
				}
			}
		} else {							/* SDv1 or MMCv3 */
			if (send_cmd(ACMD41, 0) <= 1) 	{
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
			}
			for (tmr = 8200; tmr; tmr--) {			/* Wait for leaving idle state */
				if (send_cmd(cmd, 0) == 0) break;
			}
			if (!tmr || send_cmd(CMD16, 512) != 0)	/* Set R/W block length to 512 */
				ty = 0;
		}
	}
	
	CardType = ty;
	release_spi();
	
	if (ty) interface_speed(INTERFACE_FAST);
	else power_off();
	
	s = ty ? 0 : STA_NOINIT;
	Stat = s;

	return s;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,			/* Physical drive number (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	DWORD sector,		/* Start sector number (LBA) */
	BYTE count			/* Sector count (1..255) */
)
{
	if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY;
	if (!count) return RES_PARERR;
	if (!(CardType & CT_BLOCK)) sector *=512 ; /* get address of sector */

	if (count == 1) {	/* Single block read */
		if (send_cmd(CMD17, sector) == 0)	{ /* READ_SINGLE_BLOCK */
			if (rcvr_datablock(buff, 512)) {
				count = 0;
			}
		}
	}
	else {				/* Multiple block read */
		if (send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512)) {
					break;
				}
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
			}
	}
	release_spi();
	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
#if _FS_READONLY == 0
DRESULT disk_write (
	BYTE  drv,			/* Physical drive number (0) */
	const BYTE  *buff,	/* Pointer to the data to be written */
	DWORD  sector,		/* Start sector number (LBA) */
	BYTE  count			/* Sector count (1..255) */
)
{	
	
	if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY;
	if (!count) return RES_PARERR;
	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert LBA to byte address if needed */

	if (count == 1) {	/* Single block write */
		if ((send_cmd(CMD24, sector) == 0) && (xmit_datablock(buff, 0xFE))) /* WRITE_BLOCK */
				count = 0;
	}
	else {				/* Multiple block write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}
	release_spi();
	return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY == 0 */
/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
#if _USE_IOCTL != 0
DRESULT disk_ioctl (
   BYTE drv,      /* Physical drive number (0) */
   BYTE ctrl,      /* Control code */
   void *buff      /* Buffer to send/receive control data */
)
{
   DRESULT res;
   BYTE n, csd[16], *ptr = buff;
   WORD csize;


   if (drv) return RES_PARERR;

   res = RES_ERROR;

   if (ctrl == CTRL_POWER) {
      switch (*ptr) {
//       case 0:      /* Sub control code == 0 (POWER_OFF) */
//          if (chk_power())
//             power_off();      /* Power off */
//          res = RES_OK;
//          break;
      case 1:      /* Sub control code == 1 (POWER_ON) */
         power();            /* Power on */
         res = RES_OK;
         break;
//       case 2:      /* Sub control code == 2 (POWER_GET) */
//          *(ptr+1) = (BYTE)chk_power();
//          res = RES_OK;
//          break;
      default :
         res = RES_PARERR;
      }
   }
   else {
      if (Stat & STA_NOINIT) return RES_NOTRDY;

      switch (ctrl) {
      case CTRL_SYNC :      /* Make sure that no pending write process */
         CS_L();
         if (wait_ready() == 0xFF)
            res = RES_OK;
         break;

      case GET_SECTOR_COUNT :   /* Get number of sectors on the disk (DWORD) */
         if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
            if ((csd[0] >> 6) == 1) {   /* SDC ver 2.00 */
               csize = csd[9] + ((WORD)csd[8] << 8) + 1;
               *(DWORD*)buff = (DWORD)csize << 10;
            } else {               /* SDC ver 1.XX or MMC*/
               n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
               csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
               *(DWORD*)buff = (DWORD)csize << (n - 9);
            }
            res = RES_OK;
         }
         break;

      case GET_SECTOR_SIZE :   /* Get R/W sector size (WORD) */
         *(WORD*)buff = 512;
         res = RES_OK;
         break;

      case GET_BLOCK_SIZE :   /* Get erase block size in unit of sector (DWORD) */
         if (CardType & CT_SD2) {   /* SDC ver 2.00 */
            if (send_cmd(ACMD13, 0) == 0) {   /* Read SD status */
               rcvr_spi();
               if (rcvr_datablock(csd, 16)) {            /* Read partial block */
                  for (n = 64 - 16; n; n--) rcvr_spi();   /* Purge trailing data */
                  *(DWORD*)buff = 16UL << (csd[10] >> 4);
                  res = RES_OK;
               }
            }
         } else {               /* SDC ver 1.XX or MMC */
            if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {   /* Read CSD */
               if (CardType & CT_SD1) {   /* SDC ver 1.XX */
                  *(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
               } else {               /* MMC */
                  *(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
               }
               res = RES_OK;
            }
         }
         break;

      case MMC_GET_TYPE :      /* Get card type flags (1 byte) */
         *ptr = CardType;
         res = RES_OK;
         break;

      case MMC_GET_CSD :      /* Receive CSD as a data block (16 bytes) */
         if (send_cmd(CMD9, 0) == 0      /* READ_CSD */
            && rcvr_datablock(ptr, 16))
            res = RES_OK;
         break;

      case MMC_GET_CID :      /* Receive CID as a data block (16 bytes) */
         if (send_cmd(CMD10, 0) == 0      /* READ_CID */
            && rcvr_datablock(ptr, 16))
            res = RES_OK;
         break;

      case MMC_GET_OCR :      /* Receive OCR as an R3 resp (4 bytes) */
         if (send_cmd(CMD58, 0) == 0) {   /* READ_OCR */
            for (n = 4; n; n--) *ptr++ = rcvr_spi();
            res = RES_OK;
         }
         break;

      case MMC_GET_SDSTAT :   /* Receive SD status as a data block (64 bytes) */
         if (send_cmd(ACMD13, 0) == 0) {   /* SD_STATUS */
            rcvr_spi();
            if (rcvr_datablock(ptr, 64))
               res = RES_OK;
         }
         break;

      default:
         res = RES_PARERR;
      }

      release_spi();
   }

   return res;
}
#endif /* _USE_IOCTL != 0 */




// DRESULT disk_ioctl (
// 	BYTE drv,		/* Physical drive nmuber (0) */
// 	BYTE ctrl,		/* Control code */
// 	void *buff		/* Buffer to send/receive control data */
// )
// {
// 	DRESULT res;
// 	BYTE n, csd[16];
// 	DWORD cs;


// 	if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY;	/* Check if card is in the socket */

// 	res = RES_ERROR;
// 	switch (ctrl) {
// 		case CTRL_SYNC :		/* Make sure that no pending write process */
// 			if (select()) {
// 				deselect();
// 				res = RES_OK;
// 			}
// 			break;

// 		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
// 			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
// 				if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
// 					cs = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 8) + 1;
// 					*(DWORD*)buff = cs << 10;
// 				} else {					/* SDC ver 1.XX or MMC */
// 					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
// 					cs = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
// 					*(DWORD*)buff = cs << (n - 9);
// 				}
// 				res = RES_OK;
// 			}
// 			break;

// 		case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
// 			*(DWORD*)buff = 128;
// 			res = RES_OK;
// 			break;

// 		default:
// 			res = RES_PARERR;
// 	}

// 	deselect();

// 	return res;
// }





// #if (STM32_SD_DISK_IOCTRL == 1)
// DRESULT disk_ioctl (
// 	BYTE drv,		/* Physical drive number (0) */
// 	BYTE ctrl,		/* Control code */
// 	void *buff		/* Buffer to send/receive control data */
// )
// {
// 	DRESULT res;
// 	BYTE n, csd[16], *ptr = buff;
// 	WORD csize;

// 	if (drv) return RES_PARERR;

// 	res = RES_ERROR;

// 	if (ctrl == CTRL_SYNC) {
// 		switch (*ptr) {
// 		case 0:		/* Sub control code == 0 (POWER_OFF) */
// 			//if (chk_power())
// 				power_off();		/* Power off */
// 			res = RES_OK;
// 			break;
// 		case 1:		/* Sub control code == 1 (POWER_ON) */
// 			power();				/* Power on */
// 			res = RES_OK;
// 			break;
// 		case 2:		/* Sub control code == 2 (POWER_GET) */
// 			//*(ptr+1) = (BYTE)chk_power();
// 			res = RES_OK;
// 			break;
// 		default :
// 			res = RES_PARERR;
// 		}
// 	}
// 	else {
// 		if (Stat & STA_NOINIT) return RES_NOTRDY;

// 		switch (ctrl) {
// 		case CTRL_SYNC :		/* Make sure that no pending write process */
// 			CS_L();
// 			if (wait_ready() == 0xFF)
// 				res = RES_OK;
// 			break;

// 		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
// 			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
// 				if ((csd[0] >> 6) == 1) {	/* SDC version 2.00 */
// 					csize = csd[9] + ((WORD)csd[8] << 8) + 1;
// 					*(DWORD*)buff = (DWORD)csize << 10;
// 				} else {					/* SDC version 1.XX or MMC*/
// 					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
// 					csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
// 					*(DWORD*)buff = (DWORD)csize << (n - 9);
// 				}
// 				res = RES_OK;
// 			}
// 			break;

// 		case GET_SECTOR_SIZE :	/* Get R/W sector size (WORD) */
// 			*(WORD*)buff = 512;
// 			res = RES_OK;
// 			break;

// 		case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
// 			if (CardType & CT_SD2) {	/* SDC version 2.00 */
// 				if (send_cmd(ACMD13, 0) == 0) {	/* Read SD status */
// 					rcvr_spi();
// 					if (rcvr_datablock(csd, 16)) {				/* Read partial block */
// 						for (n = 64 - 16; n; n--) rcvr_spi();	/* Purge trailing data */
// 						*(DWORD*)buff = 16UL << (csd[10] >> 4);
// 						res = RES_OK;
// 					}
// 				}
// 			} else {					/* SDC version 1.XX or MMC */
// 				if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
// 					if (CardType & CT_SD1) {	/* SDC version 1.XX */
// 						*(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
// 					} else {					/* MMC */
// 						*(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
// 					}
// 					res = RES_OK;
// 				}
// 			}
// 			break;

// // 		case MMC_GET_TYPE :		/* Get card type flags (1 byte) */
// // 			*ptr = CardType;
// // 			res = RES_OK;
// // 			break;

// // 		case MMC_GET_CSD :		/* Receive CSD as a data block (16 bytes) */
// // 			if (send_cmd(CMD9, 0) == 0		/* READ_CSD */
// // 				&& rcvr_datablock(ptr, 16))
// // 				res = RES_OK;
// // 			break;

// // 		case MMC_GET_CID :		/* Receive CID as a data block (16 bytes) */
// // 			if (send_cmd(CMD10, 0) == 0		/* READ_CID */
// // 				&& rcvr_datablock(ptr, 16))
// // 				res = RES_OK;
// // 			break;

// // 		case MMC_GET_OCR :		/* Receive OCR as an R3 resp (4 bytes) */
// // 			if (send_cmd(CMD58, 0) == 0) {	/* READ_OCR */
// // 				for (n = 4; n; n--) *ptr++ = rcvr_spi();
// // 				res = RES_OK;
// // 			}
// // 			break;

// // 		case MMC_GET_SDSTAT :	/* Receive SD status as a data block (64 bytes) */
// // 			if (send_cmd(ACMD13, 0) == 0) {	/* SD_STATUS */
// // 				rcvr_spi();
// // 				if (rcvr_datablock(ptr, 64))
// // 					res = RES_OK;
// // 			}
// // 			break;

// 		default:
// 			res = RES_PARERR;
// 		}

// 		release_spi();
// 	}

// 	return res;
// }
// #endif /* _USE_IOCTL != 0 */








// /*-------------------------------------------------------------*/
// #include <stm32f4xx.h>
// #include "diskio.h"
// #include "powersd.h"
// #include "stm32f4xx_gpio.h"
// #include "stm32f4xx_rcc.h"
// #include "stm32f4xx_spi.h"
// #include "stm32f4xx_dma.h"
// /*-----------------------------------------------------*/
// /* Hardware Configuration															 */
// /*-----------------------------------------------------*/

//  #define SPI_SD                   SPI1//
//  #define RCC_GPIO_CS              RCC_AHB1Periph_GPIOA//
//  #define GPIO_CS                  GPIOA//
//  #define GPIO_SPI_SD              GPIOA//
//  #define GPIO_Pin_CS              GPIO_Pin_4//
//  #define GPIO_Pin_SPI_SD_SCK      GPIO_Pin_5//
//  #define GPIO_Pin_SPI_SD_MISO     GPIO_Pin_6//
//  #define GPIO_Pin_SPI_SD_MOSI     GPIO_Pin_7//
//  #define RCC_SPI_SD               RCC_APB2Periph_SPI1//
//  /* - for SPI1 and full-speed APB2: 168MHz/2 */
//  #define SPI_BaudRatePrescaler_SPI_SD  SPI_BaudRatePrescaler_8 //

// /* Definitions for MMC/SDC command */
// #define CMD0	(0x40+0)	/* GO_IDLE_STATE */
// #define CMD1	(0x40+1)	/* SEND_OP_COND (MMC) */
// #define ACMD41	(0xC0+41)	/* SEND_OP_COND (SDC) */
// #define CMD8 	(0x40+8)	/* SEND_IF_COND */
// #define CMD9 	(0x40+9)	/* SEND_CSD */
// #define CMD10	(0x40+10)	/* SEND_CID */
// #define CMD12	(0x40+12)	/* STOP_TRANSMISSION */
// #define CMD13	(0x40+13)	/* SEND_STATUS */
// #define ACMD13	(0xC0+13)	/* SD_STATUS (SDC) */
// #define CMD16	(0x40+16)	/* SET_BLOCKLEN */
// #define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
// #define CMD18	(0x40+18)	/* READ_MULTIPLE_BLOCK */
// #define CMD23	(0x40+23)	/* SET_BLOCK_COUNT (MMC) */
// #define ACMD23	(0xC0+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
// #define CMD24	(0x40+24)	/* WRITE_BLOCK */
// #define CMD25	(0x40+25)	/* WRITE_MULTIPLE_BLOCK */
// #define CMD55	(0x40+55)	/* APP_CMD */
// #define CMD58	(0x40+58)	/* READ_OCR */

// #define CS_L()        GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS)    /* MMC CS = L */	//
// #define CS_H()      GPIO_SetBits(GPIO_CS, GPIO_Pin_CS)      /* MMC CS = H */		//

// /* Card type flags (CardType) */
// // #define CT_MMC		0x01		/* MMC ver 3 */
// // #define CT_SD1		0x02		/* SD ver 1 */
// // #define CT_SD2		0x04		/* SD ver 2 */
// // #define CT_SDC		0x06		/* SD */
// // #define CT_BLOCK	0x08		/* Block addressing */
// /* Manley EK-STM32F board does not offer socket contacts -> dummy values: */
// #define SOCKPORT	1			/* Socket contact port */
// #define SOCKWP		0			/* Write protect switch (PB5) */
// #define SOCKINS		0			/* Card detect switch (PB4) */

// #if (_MAX_SS != 512) || (_FS_READONLY == 0) || (STM32_SD_DISK_IOCTRL_FORCE == 1)
// #define STM32_SD_DISK_IOCTRL   1
// #else
// #define STM32_SD_DISK_IOCTRL   0
// #endif
// static const DWORD socket_state_mask_cp = (1 << 0);
// static const DWORD socket_state_mask_wp = (1 << 1);

// static
// DSTATUS Stat = STA_NOINIT;	/* Disk status */

// static
// BYTE CardType;			/* b0:MMC, b1:SDv1, b2:SDv2, b3:Block addressing */


// /*--------------------------------------------------------------------------

//    Module Private Functions and Variables

// ---------------------------------------------------------------------------*/
// enum speed_setting { INTERFACE_SLOW, INTERFACE_FAST };

// static void interface_speed( enum speed_setting speed)
// {
// 	uint32_t tmp;

// 	tmp = SPI_SD->CR1;
// 	if (speed== INTERFACE_SLOW ) {
// 		/* Set slow clock (100k-400k) */
// 		tmp = ( tmp | SPI_BaudRatePrescaler_256);
// 	} else {
// 		/* Set fast clock (depends on the CSD) */
// 		tmp = ( tmp & ~SPI_BaudRatePrescaler_256) | SPI_BaudRatePrescaler_SPI_SD;
// 	}
// 	SPI_SD->CR1 = tmp;
// }
// #if SOCKET_WP_CONNECTED
// /* Socket's Write-Protection Pin: high = write-protected, low = writable */

// static void socket_wp_init(void)
// {
// 	GPIO_InitTypeDef GPIO_InitStructure;

// 	/* Configure I/O for write-protect */
// 	RCC_APB2PeriphClockCmd(RCC_APBxPeriph_GPIO_WP, ENABLE);
// 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_WP;
// 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_WP;
// 	GPIO_Init(GPIO_WP, &GPIO_InitStructure);
// }

// static DWORD socket_is_write_protected(void)
// {
// 	return ( GPIO_ReadInputData(GPIO_WP) & GPIO_Pin_WP ) ? socket_state_mask_wp : 0;
// }

// #else

// static void socket_wp_init(void)
// {
// 	return;
// }
// static inline DWORD socket_is_write_protected(void)
// {
// 	return 0; /* fake not protected */
// }

// #endif /* SOCKET_WP_CONNECTED */


// #if SOCKET_CP_CONNECTED
// /* Socket's Card-Present Pin: high = socket empty, low = card inserted */

// static void socket_cp_init(void)
// {
// 	GPIO_InitTypeDef GPIO_InitStructure;

// 	/* Configure I/O for card-present */
// 	RCC_APB2PeriphClockCmd(RCC_APBxPeriph_GPIO_CP, ENABLE);
// 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_CP;
// 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_CP;
// 	GPIO_Init(GPIO_CP, &GPIO_InitStructure);
// }

// static inline DWORD socket_is_empty(void)
// {
// 	return ( GPIO_ReadInputData(GPIO_CP) & GPIO_Pin_CP ) ? socket_state_mask_cp : FALSE;
// }

// #else

// static void socket_cp_init(void)
// {
// 	return;
// }

// static inline DWORD socket_is_empty(void)
// {
// 	return 0; /* fake inserted */
// }

// #endif /* SOCKET_CP_CONNECTED */


// #if CARD_SUPPLY_SWITCHABLE

// static void card_power(BOOL on)		/* switch FET for card-socket VCC */
// {
// 	GPIO_InitTypeDef GPIO_InitStructure;

// 	/* Turn on GPIO for power-control pin connected to FET's gate */
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_PWR, ENABLE);
// 	/* Configure I/O for Power FET */
// 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_PWR;
// 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_PWR;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIO_PWR, &GPIO_InitStructure);
// 	if (on) {
// 		GPIO_ResetBits(GPIO_PWR, GPIO_Pin_PWR);
// 	} else {
// 		/* Chip select internal pull-down (to avoid parasite powering) */
// 		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CS;
// 		GPIO_Init(GPIO_CS, &GPIO_InitStructure);

// 		GPIO_SetBits(GPIO_PWR, GPIO_Pin_PWR);
// 	}
// }

// #if (STM32_SD_DISK_IOCTRL == 1)
// static int chk_power(void)		/* Socket power state: 0=off, 1=on */
// {
// 	if ( GPIO_ReadOutputDataBit(GPIO_PWR, GPIO_Pin_PWR) == Bit_SET ) {
// 		return 0;
// 	} else {
// 		return 1;
// 	}
// }
// #endif

// #else

// static void card_power(BYTE on)
// {
// 	on=on;
// }

// #if (STM32_SD_DISK_IOCTRL == 1)
// static int chk_power(void)
// {
// 	return 1; /* fake powered */
// }
// #endif

// #endif /* CARD_SUPPLY_SWITCHABLE */

// /*-----------------------------------------------------------------------*/
// /* Transmit/Receive a byte to MMC via SPI  (Platform dependent)          */
// /*-----------------------------------------------------------------------*/
// static uint8_t stm32_spi_rw(  uint8_t out )
// {
// 	/* Loop while DR register in not empty */
// 	// while (SPI_I2S_GetFlagStatus(SPI_SD, SPI_I2S_FLAG_TXE) == RESET) { ; }

// 	/* Send byte through the SPI peripheral */
// 	SPI_I2S_SendData(SPI_SD, out);
// 	/* Loop while DR register is empty */
// 	/* Wait to receive a byte */
// 	while (SPI_I2S_GetFlagStatus(SPI_SD, SPI_I2S_FLAG_RXNE) == RESET) { ; }

// 	/* Return the byte read from the SPI bus */
// 	return SPI_I2S_ReceiveData(SPI_SD);
// }



// /*-----------------------------------------------------------------------*/
// /* Transmit a byte to MMC via SPI  (Platform dependent)                  */
// /*-----------------------------------------------------------------------*/

// #define xmit_spi(dat)  stm32_spi_rw(dat)

// /*-----------------------------------------------------------------------*/
// /* Receive a byte from MMC via SPI  (Platform dependent)                 */
// /*-----------------------------------------------------------------------*/
// /* Receive a response when sent oxff*/
// static uint8_t rcvr_spi (void)
// {
// 	return stm32_spi_rw(0xff);
// }

// /* Alternative macro to receive data fast */
// /* dest is address of the memory where held response*/
// #define rcvr_spi_m(dst)  *(dst)=stm32_spi_rw(0xff)

// /*-----------------------------------------------------------------------*/
// /* Wait for card ready                                                   */
// /*-----------------------------------------------------------------------*/

// static int wait_ready (void)
// {
// 	BYTE res;
// 	UINT i;
// 	i = 82000;
// 	
// 	rcvr_spi(); /* Tranmit oxff and receive response by stm32_spi_rw (oxff) */
// 	do{
// 		res = rcvr_spi();
// 		i-- ;
// 	}
// 	while ((res != 0xFF) && i>0);
// 	return i? 1:0;
// }

// /*-----------------------------------------------------------------------*/
// /* Deselect the card and release SPI bus                                 */
// /*-----------------------------------------------------------------------*/

// static void deselect (void)
// {
// 	CS_H();
// 	rcvr_spi();	/* Dummy clock (force DO hi-z for multiple slave SPI) */
// }

// /*-----------------------------------------------------------------------*/
// /* Select the card and wait for ready                                    */
// /*-----------------------------------------------------------------------*/

// static int select (void)	/* 1:OK, 0:Timeout */
// {
// 	CS_L();
// 	rcvr_spi();	/* Dummy clock (force DO enabled) */

// 	if (wait_ready()) return 1;	/* OK */
// 	deselect();
// 	return 0;			/* Failed */
// }

// /*-----------------------------------------------------------------------*/
// /* Deselect the card and release SPI bus                                 */
// /*-----------------------------------------------------------------------*/

// static
// void release_spi (void)
// {
// 	deselect(); /* CS= HIGHT*/
// 	rcvr_spi();   /* Need 8 cycle clocks to  succesively finish*/
// }

// /*-----------------------------------------------------------------------*/
// /* Power Control and interface-initialization (Platform dependent)       */
// /*-----------------------------------------------------------------------*/
// #ifdef STM32_SD_USE_DMA
// /*-----------------------------------------------------------------------*/
// /* Transmit/Receive Block using DMA (Platform dependent. STM32 here)     */
// /*-----------------------------------------------------------------------*/
// static
// void stm32_dma_transfer(
// 	BOOL receive,		/* FALSE for buff->SPI, TRUE for SPI->buff               */
// 	const BYTE *buff,	/* receive TRUE  : 512 byte data block to be transmitted
// 						   receive FALSE : Data buffer to store received data    */
// 	UINT btr 			/* receive TRUE  : Byte count (must be multiple of 2)
// 						   receive FALSE : Byte count (must be 512)              */
// )
// {
// 	DMA_InitTypeDef DMA_InitStructure;
// 	WORD rw_workbyte[] = { 0xffff };
// 	DMA_DeInit(DMA2_Stream0);//SPI-RX
// 	DMA_DeInit(DMA2_Stream5);//SPI-TX
// 	//RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_DMA2, ENABLE); 
// 	DMA_InitStructure.DMA_Channel        = DMA_Channel_3;
// 	/* shared DMA configuration values */
// 	DMA_InitStructure.DMA_PeripheralBaseAddr = (DWORD)(&(SPI_SD->DR));
// 	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
// 	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
// 	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// 	DMA_InitStructure.DMA_BufferSize = btr;
// 	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
// 	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
// 	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
// 	DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_Full;
// 	DMA_InitStructure.DMA_MemoryBurst          = DMA_MemoryBurst_Single;
// 	DMA_InitStructure.DMA_PeripheralBurst       = DMA_PeripheralBurst_Single;
// 	//DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

// // 	DMA_DeInit(DMA2_Stream0);//SPI-RX
// // 	DMA_DeInit(DMA2_Stream5);//SPI-TX

// 	if ( receive ) {

// 		/* DMA2 channel3 configuration SPI1 RX ---------------------------------------------*/
// 		/* DMA2 channel3 configuration SPI2 RX ---------------------------------------------*/
// 		DMA_InitStructure.DMA_MemoryBaseAddr = (DWORD)buff;
// 		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
// 		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
// 		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
// 		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
// 		DMA_Init(DMA2_Stream0, &DMA_InitStructure);

// 		/* DMA1 channel3 configuration SPI1 TX ---------------------------------------------*/
// 		/* DMA1 channel5 configuration SPI2 TX ---------------------------------------------*/
// 		DMA_InitStructure.DMA_MemoryBaseAddr = (DWORD)rw_workbyte;
// 		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
// 		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
// 		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
// 		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
// 		DMA_Init(DMA2_Stream5, &DMA_InitStructure);

// 	} else {

// #if _FS_READONLY == 0
// 		/* DMA1 channel2 configuration SPI1 RX ---------------------------------------------*/
// 		/* DMA1 channel4 configuration SPI2 RX ---------------------------------------------*/
// 		DMA_InitStructure.DMA_MemoryBaseAddr = (DWORD)rw_workbyte;
// 		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
// 		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
// 		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
// 		DMA_Init(DMA2_Stream0, &DMA_InitStructure);

// 		/* DMA1 channel3 configuration SPI1 TX ---------------------------------------------*/
// 		/* DMA1 channel5 configuration SPI2 TX ---------------------------------------------*/
// 		DMA_InitStructure.DMA_MemoryBaseAddr = (DWORD)buff;
// 		//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
// 		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
// 		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
// 		DMA_Init(DMA2_Stream5, &DMA_InitStructure);
// #endif

// 	}

// 	/* Enable DMA RX Channel */
// 	DMA_Cmd(DMA2_Stream0, ENABLE);
// 	/* Enable DMA TX Channel */
// 	DMA_Cmd(DMA2_Stream5, ENABLE);

// 	/* Enable SPI TX/RX request */
// 	SPI_I2S_DMACmd(SPI_SD, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

// 	/* Wait until DMA1_Channel 3 Transfer Complete */
// 	/// not needed: while (DMA_GetFlagStatus(DMA_FLAG_SPI_SD_TC_TX) == RESET) { ; }
// 	/* Wait until DMA1_Channel 2 Receive Complete */
// 	//while (DMA_GetFlagStatus(DMA_FLAG_SPI_SD_TC_RX) == RESET) { ; }
// 	// same w/o function-call:
// 	// while ( ( ( DMA1->ISR ) & DMA_FLAG_SPI_SD_TC_RX ) == RESET ) { ; }
// /* Wait until DMA1_Channel 4 Receive Complete */
//    while (DMA_GetFlagStatus(DMA2_Stream0,DMA_FLAG_TCIF3) == RESET) { ; }
// 	/* Disable DMA RX Channel */
// 	DMA_Cmd(DMA2_Stream0, DISABLE);
// 	/* Disable DMA TX Channel */
// 	DMA_Cmd(DMA2_Stream5, DISABLE);

// 	/* Disable SPI RX/TX request */
// 	SPI_I2S_DMACmd(SPI_SD, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);
// 	//DMA_ClearFlag(DMA2_Stream0,DMA_FLAG_TCIF2);
//   DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TCIF5);
// }
// #endif /* STM32_SD_USE_DMA */


// static
// void power_off (void)
// {
// 	GPIO_InitTypeDef GPIO_InitStructure;

// 	select();
// 	wait_ready();/* escape if receive is oxff or time2 is out*/
// 	release_spi(); /* CS = HIGH*/

// 	SPI_I2S_DeInit(SPI_SD);
// 	SPI_Cmd(SPI_SD, DISABLE);
// 	RCC_APB2PeriphClockCmd(RCC_SPI_SD, DISABLE);

// 	/* All SPI-Pins to input with weak internal pull-downs */
// 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_SPI_SD_SCK | GPIO_Pin_SPI_SD_MISO | GPIO_Pin_SPI_SD_MOSI;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
// 	GPIO_Init(GPIO_SPI_SD, &GPIO_InitStructure);
// 	Stat |= STA_NOINIT;
// }


// /*-----------------------------------------------------------------------*/
// /* Receive a data packet from MMC                                        */
// /*-----------------------------------------------------------------------*/

// static int rcvr_datablock ( /* 1:OK, 0:Failed */
//  	BYTE *buff,			/* Data buffer to store received data */
// 	UINT  btr			/* Byte count (must be multiple of 4) */
// )
// {
// 	BYTE token;
// 	UINT i;

// 	i=9000;
// 	do {							/* Wait for data packet in timeout of 100ms */
// 		token = rcvr_spi();		/* Wait for valid response oxfe*/
// 		i--;
// 	} while ((token == 0xFF) && i>0); /*if escape, token = oxff if timer1 is out, else token=oxfe*/
// 	if(token != 0xFE) return 0;	/* If not valid data token, return with error */
// 	
// // 	do {							/* Receive the data block into buffer */
// // 		rcvr_spi_m(buff++);
// // 		rcvr_spi_m(buff++);
// // 		rcvr_spi_m(buff++);
// // 		rcvr_spi_m(buff++);
// // 	} while (btr -= 4);
// #ifdef STM32_SD_USE_DMA
// stm32_dma_transfer( TRUE, buff, btr );
// #else
// do {							/* Receive the data block into buffer */
// 	rcvr_spi_m(buff++);
// 	rcvr_spi_m(buff++);
// 	rcvr_spi_m(buff++);
// 	rcvr_spi_m(buff++);
// } while (btr -= 4);
// #endif /* STM32_SD_USE_DMA */

// 	rcvr_spi();						/* Discard CRC */
// 	rcvr_spi();

// 	return 1;					/* Return with success */
// }



// /*-----------------------------------------------------------------------*/
// /* Send a data packet to MMC                                             */
// /*-----------------------------------------------------------------------*/
// /* No use in my thesic */
// #if _FS_READONLY == 0
// static int xmit_datablock ( /* 1:OK, 0:Failed */
// 	const BYTE *buff,	/* 512 byte data block to be transmitted */
// 	BYTE  token			/* Data/Stop token */
// )
// {
// 	BYTE resp;
// 	BYTE  wc;
// 	if (!wait_ready()) return 0;

// 	xmit_spi(token);					/* transmit data token */
// 	if (token != 0xFD) {		/* Is data token */

// 		wc = 0;
// // 		do {							/* transmit the 512 byte data block to MMC */
// // 			xmit_spi(*buff++);
// // 			xmit_spi(*buff++);
// // 		} while (--wc);
// // #ifndef STM32_SD_USE_DMA
// // 	BYTE wc;
// // #endif

// // 	if (wait_ready() != 0xFF) return 0;

// // 	xmit_spi(token);					/* transmit data token */
// // 	if (token != 0xFD) {	/* Is data token */

// #ifdef STM32_SD_USE_DMA
// 		stm32_dma_transfer( FALSE, buff, 512 );
// #else
// 		wc = 0;
// 		do {							/* transmit the 512 byte data block to MMC */
// 			xmit_spi(*buff++);
// 			xmit_spi(*buff++);
// 		} while (--wc);
// #endif /* STM32_SD_USE_DMA */

// 		xmit_spi(0xFF);					/* CRC (Dummy) */
// 		xmit_spi(0xFF);
// 		resp = rcvr_spi();				/* Receive data response */
// 		if ((resp & 0x1F) != 0x05) {		/* If not accepted, return with error */
// 			return 0;
// 		}
// 	}						/* illegal command and in idle state*/
// 	return 1;
// }
// #endif /* _READONLY */


// /*-----------------------------------------------------------------------*/
// /* Send a command packet to MMC                                          */
// /*-----------------------------------------------------------------------*/

// static BYTE send_cmd (
// 	BYTE cmd,		/* Command byte */
// 	DWORD arg		/* Argument  usually is address*/
// )
// {
// 	BYTE n, res;
// 	
// 	if (cmd & 0x80) {	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
// 		cmd &= 0x7F;
// 		n = send_cmd(CMD55, 0);
// 		if (n > 1) return n;
// 	}

// 	/* Select the card and wait for ready */
// 	deselect();
// 	if(!select()) return 0xFF;
// 	
// 	/* Send command packet */
// 	xmit_spi(cmd);						/* Start + Command index */
// 	xmit_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
// 	xmit_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
// 	xmit_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
// 	xmit_spi((BYTE)arg);				/* Argument[7..0] */
// 	n = 0x01;							/* Dummy CRC + Stop */
// 	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
// 	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
// 	xmit_spi(n);

// 	/* Receive command response */
// 	if (cmd == CMD12) rcvr_spi();		/* Skip a stuff byte when stop reading */
// 	
// 	n = 10;								/* Wait for a valid response in timeout of 10 attempts */
// 	do
// 	{
// 		res = rcvr_spi();
// 	}
// 	while ((res & 0x80) && --n); /* wait for valid respose*/

// 	return res;			/* Return with the response value */
// }



// /*--------------------------------------------------------------------------

//    Public Functions

// ---------------------------------------------------------------------------*/

// /*-----------------------------------------------------------------------*/
// /* Get Disk Status                                                       */
// /*-----------------------------------------------------------------------*/

// DSTATUS disk_status (
// 	BYTE drv			/* Drive number (always 0) */
// )
// {
// 	DSTATUS s;


// 	if (drv) return STA_NOINIT;

// 	/* Check if the card is kept initialized */
// 	s = Stat;
// 	if (!(s & STA_NOINIT)) {
// 		if (send_cmd(CMD13, 0))	/* Read card status */
// 			s = STA_NOINIT;
// 		rcvr_spi();		/* Receive following half of R2 */
// 		deselect();
// 	}
// 	Stat = s;

// 	return s;
// }

// /*-----------------------------------------------------------------------*/
// /* Initialize Disk Drive                                                 */
// /*-----------------------------------------------------------------------*/

// DSTATUS disk_initialize (
// 	BYTE drv		/* Physical drive nmuber (0) */
// )
// {
// 	BYTE n, ty, cmd, buf[4];
// 	UINT tmr;
// 	DSTATUS s;


// 	if (drv) return RES_NOTRDY;

// 	interface_speed(INTERFACE_SLOW);
// 	
// 	for (n = 10; n; n--) rcvr_spi();	/* 80 dummy clocks */	

// 	ty = 0;
// 	if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
// 		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? */
// 			rcvr_spi();				/* Get trailing return value of R7 resp */
// 			rcvr_spi();
// 			buf[2] = rcvr_spi();
// 			buf[3] = rcvr_spi();
// 			if (buf[2] == 0x01 && buf[3] == 0xAA) {		/* The card can work at vdd range of 2.7-3.6V */
// 				for (tmr = 8200; tmr; tmr--) {			/* Wait for leaving idle state (ACMD41 with HCS bit) */
// 					if (send_cmd(ACMD41, 1UL << 30) == 0) break;
// 				}
// 				if (tmr && send_cmd(CMD58, 0) == 0) {	/* Check CCS bit in the OCR */
// 					//buf[0] = rcvr_spi();
// /*.....................................................................................................................*/
// 					while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
// 					SPI1->DR = 0xFF;
// 					/* Wait for SPI2 data reception */
// 					while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
// 					buf[0] = SPI1->DR;
// /*.....................................................................................................................*/
// 					rcvr_spi();
// 					rcvr_spi();
// 					rcvr_spi();
// 					ty = (buf[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 */
// 				}
// 			}
// 		} else {							/* SDv1 or MMCv3 */
// 			if (send_cmd(ACMD41, 0) <= 1) 	{
// 				ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
// 			} else {
// 				ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
// 			}
// 			for (tmr = 8200; tmr; tmr--) {			/* Wait for leaving idle state */
// 				if (send_cmd(cmd, 0) == 0) break;
// 			}
// 			if (!tmr || send_cmd(CMD16, 512) != 0)	/* Set R/W block length to 512 */
// 				ty = 0;
// 		}
// 	}
// 	
// 	CardType = ty;
// 	release_spi();
// 	
// 	if (ty) interface_speed(INTERFACE_FAST);
// 	else power_off();
// 	
// 	s = ty ? 0 : STA_NOINIT;
// 	Stat = s;

// 	return s;
// }



// /*-----------------------------------------------------------------------*/
// /* Read Sector(s)                                                        */
// /*-----------------------------------------------------------------------*/

// DRESULT disk_read (
// 	BYTE drv,			/* Physical drive number (0) */
// 	BYTE *buff,			/* Pointer to the data buffer to store read data */
// 	DWORD sector,		/* Start sector number (LBA) */
// 	BYTE count			/* Sector count (1..255) */
// )
// {
// 	if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY;
// 	if (!count) return RES_PARERR;
// 	if (!(CardType & CT_BLOCK)) sector *=512 ; /* get address of sector */

// 	if (count == 1) {	/* Single block read */
// 		if (send_cmd(CMD17, sector) == 0)	{ /* READ_SINGLE_BLOCK */
// 			if (rcvr_datablock(buff, 512)) {
// 				count = 0;
// 			}
// 		}
// 	}
// 	else {				/* Multiple block read */
// 		if (send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
// 			do {
// 				if (!rcvr_datablock(buff, 512)) {
// 					break;
// 				}
// 				buff += 512;
// 			} while (--count);
// 			send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
// 			}
// 	}
// 	release_spi();
// 	return count ? RES_ERROR : RES_OK;
// }



// /*-----------------------------------------------------------------------*/
// /* Write Sector(s)                                                       */
// /*-----------------------------------------------------------------------*/
// #if _FS_READONLY == 0
// DRESULT disk_write (
// 	BYTE  drv,			/* Physical drive number (0) */
// 	const BYTE  *buff,	/* Pointer to the data to be written */
// 	DWORD  sector,		/* Start sector number (LBA) */
// 	BYTE  count			/* Sector count (1..255) */
// )
// {	
// 	
// 	if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY;
// 	if (!count) return RES_PARERR;
// 	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert LBA to byte address if needed */

// 	if (count == 1) {	/* Single block write */
// 		if ((send_cmd(CMD24, sector) == 0) && (xmit_datablock(buff, 0xFE))) /* WRITE_BLOCK */
// 				count = 0;
// 	}
// 	else {				/* Multiple block write */
// 		if (CardType & CT_SDC) send_cmd(ACMD23, count);
// 		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
// 			do {
// 				if (!xmit_datablock(buff, 0xFC)) break;
// 				buff += 512;
// 			} while (--count);
// 			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
// 				count = 1;
// 		}
// 	}
// 	release_spi();
// 	return count ? RES_ERROR : RES_OK;
// }
// #endif /* _READONLY == 0 */
// /*-----------------------------------------------------------------------*/
// /* Miscellaneous Functions                                               */
// /*-----------------------------------------------------------------------*/
// #if _USE_IOCTL != 0
// DRESULT disk_ioctl (
//    BYTE drv,      /* Physical drive number (0) */
//    BYTE ctrl,      /* Control code */
//    void *buff      /* Buffer to send/receive control data */
// )
// {
//    DRESULT res;
//    BYTE n, csd[16], *ptr = buff;
//    WORD csize;


//    if (drv) return RES_PARERR;

//    res = RES_ERROR;

//    if (ctrl == CTRL_POWER) {
//       switch (*ptr) {
//       case 0:      /* Sub control code == 0 (POWER_OFF) */
//          if (chk_power())
//             power_off();      /* Power off */
//          res = RES_OK;
//          break;
//       case 1:      /* Sub control code == 1 (POWER_ON) */
//          power();            /* Power on */
//          res = RES_OK;
//          break;
//       case 2:      /* Sub control code == 2 (POWER_GET) */
//          *(ptr+1) = (BYTE)chk_power();
//          res = RES_OK;
//          break;
//       default :
//          res = RES_PARERR;
//       }
//    }
//    else {
//       if (Stat & STA_NOINIT) return RES_NOTRDY;

//       switch (ctrl) {
//       case CTRL_SYNC :      /* Make sure that no pending write process */
//          CS_L();
//          if (wait_ready() == 0xFF)
//             res = RES_OK;
//          break;

//       case GET_SECTOR_COUNT :   /* Get number of sectors on the disk (DWORD) */
//          if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
//             if ((csd[0] >> 6) == 1) {   /* SDC ver 2.00 */
//                csize = csd[9] + ((WORD)csd[8] << 8) + 1;
//                *(DWORD*)buff = (DWORD)csize << 10;
//             } else {               /* SDC ver 1.XX or MMC*/
//                n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
//                csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
//                *(DWORD*)buff = (DWORD)csize << (n - 9);
//             }
//             res = RES_OK;
//          }
//          break;

//       case GET_SECTOR_SIZE :   /* Get R/W sector size (WORD) */
//          *(WORD*)buff = 512;
//          res = RES_OK;
//          break;

//       case GET_BLOCK_SIZE :   /* Get erase block size in unit of sector (DWORD) */
//          if (CardType & CT_SD2) {   /* SDC ver 2.00 */
//             if (send_cmd(ACMD13, 0) == 0) {   /* Read SD status */
//                rcvr_spi();
//                if (rcvr_datablock(csd, 16)) {            /* Read partial block */
//                   for (n = 64 - 16; n; n--) rcvr_spi();   /* Purge trailing data */
//                   *(DWORD*)buff = 16UL << (csd[10] >> 4);
//                   res = RES_OK;
//                }
//             }
//          } else {               /* SDC ver 1.XX or MMC */
//             if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {   /* Read CSD */
//                if (CardType & CT_SD1) {   /* SDC ver 1.XX */
//                   *(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
//                } else {               /* MMC */
//                   *(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
//                }
//                res = RES_OK;
//             }
//          }
//          break;

//       case MMC_GET_TYPE :      /* Get card type flags (1 byte) */
//          *ptr = CardType;
//          res = RES_OK;
//          break;

//       case MMC_GET_CSD :      /* Receive CSD as a data block (16 bytes) */
//          if (send_cmd(CMD9, 0) == 0      /* READ_CSD */
//             && rcvr_datablock(ptr, 16))
//             res = RES_OK;
//          break;

//       case MMC_GET_CID :      /* Receive CID as a data block (16 bytes) */
//          if (send_cmd(CMD10, 0) == 0      /* READ_CID */
//             && rcvr_datablock(ptr, 16))
//             res = RES_OK;
//          break;

//       case MMC_GET_OCR :      /* Receive OCR as an R3 resp (4 bytes) */
//          if (send_cmd(CMD58, 0) == 0) {   /* READ_OCR */
//             for (n = 4; n; n--) *ptr++ = rcvr_spi();
//             res = RES_OK;
//          }
//          break;

//       case MMC_GET_SDSTAT :   /* Receive SD status as a data block (64 bytes) */
//          if (send_cmd(ACMD13, 0) == 0) {   /* SD_STATUS */
//             rcvr_spi();
//             if (rcvr_datablock(ptr, 64))
//                res = RES_OK;
//          }
//          break;

//       default:
//          res = RES_PARERR;
//       }

//       release_spi();
//    }

//    return res;
// }
// #endif /* _USE_IOCTL != 0 */
// // DRESULT disk_ioctl (
// // 	BYTE drv,		/* Physical drive nmuber (0) */
// // 	BYTE ctrl,		/* Control code */
// // 	void *buff		/* Buffer to send/receive control data */
// // )
// // {
// // 	DRESULT res;
// // 	BYTE n, csd[16];
// // 	DWORD cs;


// // 	if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY;	/* Check if card is in the socket */

// // 	res = RES_ERROR;
// // 	switch (ctrl) {
// // 		case CTRL_SYNC :		/* Make sure that no pending write process */
// // 			if (select()) {
// // 				deselect();
// // 				res = RES_OK;
// // 			}
// // 			break;

// // 		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
// // 			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
// // 				if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
// // 					cs = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 8) + 1;
// // 					*(DWORD*)buff = cs << 10;
// // 				} else {					/* SDC ver 1.XX or MMC */
// // 					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
// // 					cs = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
// // 					*(DWORD*)buff = cs << (n - 9);
// // 				}
// // 				res = RES_OK;
// // 			}
// // 			break;

// // 		case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
// // 			*(DWORD*)buff = 128;
// // 			res = RES_OK;
// // 			break;

// // 		default:
// // 			res = RES_PARERR;
// // 	}

// // 	deselect();

// // 	return res;
// // }

// // #if (STM32_SD_DISK_IOCTRL == 1)
// // DRESULT disk_ioctl (
// // 	BYTE drv,		/* Physical drive number (0) */
// // 	BYTE ctrl,		/* Control code */
// // 	void *buff		/* Buffer to send/receive control data */
// // )
// // {
// // 	DRESULT res;
// // 	BYTE n, csd[16], *ptr = buff;
// // 	WORD csize;

// // 	if (drv) return RES_PARERR;

// // 	res = RES_ERROR;

// // 	if (ctrl == CTRL_SYNC) {
// // 		switch (*ptr) {
// // 		case 0:		/* Sub control code == 0 (POWER_OFF) */
// // 			//if (chk_power())
// // 				power_off();		/* Power off */
// // 			res = RES_OK;
// // 			break;
// // 		case 1:		/* Sub control code == 1 (POWER_ON) */
// // 			power();				/* Power on */
// // 			res = RES_OK;
// // 			break;
// // 		case 2:		/* Sub control code == 2 (POWER_GET) */
// // 			//*(ptr+1) = (BYTE)chk_power();
// // 			res = RES_OK;
// // 			break;
// // 		default :
// // 			res = RES_PARERR;
// // 		}
// // 	}
// // 	else {
// // 		if (Stat & STA_NOINIT) return RES_NOTRDY;

// // 		switch (ctrl) {
// // 		case CTRL_SYNC :		/* Make sure that no pending write process */
// // 			CS_L();
// // 			if (wait_ready() == 0xFF)
// // 				res = RES_OK;
// // 			break;

// // 		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
// // 			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
// // 				if ((csd[0] >> 6) == 1) {	/* SDC version 2.00 */
// // 					csize = csd[9] + ((WORD)csd[8] << 8) + 1;
// // 					*(DWORD*)buff = (DWORD)csize << 10;
// // 				} else {					/* SDC version 1.XX or MMC*/
// // 					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
// // 					csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
// // 					*(DWORD*)buff = (DWORD)csize << (n - 9);
// // 				}
// // 				res = RES_OK;
// // 			}
// // 			break;

// // 		case GET_SECTOR_SIZE :	/* Get R/W sector size (WORD) */
// // 			*(WORD*)buff = 512;
// // 			res = RES_OK;
// // 			break;

// // 		case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
// // 			if (CardType & CT_SD2) {	/* SDC version 2.00 */
// // 				if (send_cmd(ACMD13, 0) == 0) {	/* Read SD status */
// // 					rcvr_spi();
// // 					if (rcvr_datablock(csd, 16)) {				/* Read partial block */
// // 						for (n = 64 - 16; n; n--) rcvr_spi();	/* Purge trailing data */
// // 						*(DWORD*)buff = 16UL << (csd[10] >> 4);
// // 						res = RES_OK;
// // 					}
// // 				}
// // 			} else {					/* SDC version 1.XX or MMC */
// // 				if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
// // 					if (CardType & CT_SD1) {	/* SDC version 1.XX */
// // 						*(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
// // 					} else {					/* MMC */
// // 						*(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
// // 					}
// // 					res = RES_OK;
// // 				}
// // 			}
// // 			break;

// // // 		case MMC_GET_TYPE :		/* Get card type flags (1 byte) */
// // // 			*ptr = CardType;
// // // 			res = RES_OK;
// // // 			break;

// // // 		case MMC_GET_CSD :		/* Receive CSD as a data block (16 bytes) */
// // // 			if (send_cmd(CMD9, 0) == 0		/* READ_CSD */
// // // 				&& rcvr_datablock(ptr, 16))
// // // 				res = RES_OK;
// // // 			break;

// // // 		case MMC_GET_CID :		/* Receive CID as a data block (16 bytes) */
// // // 			if (send_cmd(CMD10, 0) == 0		/* READ_CID */
// // // 				&& rcvr_datablock(ptr, 16))
// // // 				res = RES_OK;
// // // 			break;

// // // 		case MMC_GET_OCR :		/* Receive OCR as an R3 resp (4 bytes) */
// // // 			if (send_cmd(CMD58, 0) == 0) {	/* READ_OCR */
// // // 				for (n = 4; n; n--) *ptr++ = rcvr_spi();
// // // 				res = RES_OK;
// // // 			}
// // // 			break;

// // // 		case MMC_GET_SDSTAT :	/* Receive SD status as a data block (64 bytes) */
// // // 			if (send_cmd(ACMD13, 0) == 0) {	/* SD_STATUS */
// // // 				rcvr_spi();
// // // 				if (rcvr_datablock(ptr, 64))
// // // 					res = RES_OK;
// // // 			}
// // // 			break;

// // 		default:
// // 			res = RES_PARERR;
// // 		}

// // 		release_spi();
// // 	}

// // 	return res;
// // }
// // #endif /* _USE_IOCTL != 0 */























