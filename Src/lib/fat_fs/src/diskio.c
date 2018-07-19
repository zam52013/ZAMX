/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "stm32_eval_sdio_sd.h"	/* Example: Header file of existing USB MSD control module */
#include "usbh_usr.h"
/* Definitions of physical drive number for each drive */
#define SD		0	/* Example: Map ATA harddisk to physical drive 0 */
#define MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define USB		2	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case SD :
		result = SD_GetStatus();

		// translate the reslut code here

		return stat;

	case MMC :
		//result = MMC_disk_status();

		// translate the reslut code here

		return stat;

	case USB :
		//result = USB_disk_status();
		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	//DSTATUS stat;
	int result;

	switch (pdrv) {
	case SD :
		result = SD_Init();

		// translate the reslut code here

		return result;

	case MMC :
		//result = MMC_disk_initialize();

		// translate the reslut code here

		return result;

	case USB :
			if(USBH_UDISK_Status())return 0;
	else return 1;
		// translate the reslut code here

		return result;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	//DRESULT res;
	int result;

	switch (pdrv) {
	case SD :
		// translate the arguments here

		result = SD_ReadBlock(buff, sector, count);

		// translate the reslut code here

		return result;

	case MMC :
		// translate the arguments here

		//result = MMC_disk_read(buff, sector, count);

		// translate the reslut code here

		return result;

	case USB :
		// translate the arguments here

		result = USBH_UDISK_Read(buff, sector, count);

		// translate the reslut code here

		return result;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	//DRESULT res;
	int result;

	switch (pdrv) {
	case SD :
		// translate the arguments here

		result = SD_WriteBlock((u8 *)buff, sector, count);
		if(result==2)
		{
			SD_Init();
			SD_WriteBlock((u8 *)buff, sector, count);
		}
		// translate the reslut code here

		return result;

	case MMC :
		// translate the arguments here

	//	result = MMC_disk_write(buff, sector, count);

		// translate the reslut code here

		return result;

	case USB :
		// translate the arguments here

		result = USBH_UDISK_Write((u8 *)buff, sector, count);

		// translate the reslut code here

		return result;
	}

	return RES_PARERR;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case SD :
		// pre-process here

		//result = ATA_disk_ioctl(cmd, buff);

		// post-process here

		return res;

	case MMC :
		// pre-process here

		//result = MMC_disk_ioctl(cmd, buff);

		// post-process here

		return res;

	case USB :
		// pre-process here

		//result = USB_disk_ioctl(cmd, buff);

		// post-process here

		return res;
	}

	return RES_PARERR;
}
#endif

DWORD get_fattime (void)
{				 
	return 0;
}		