/* --COPYRIGHT--,
 * Copyright (c) 2021, TAPIT Engineering Co., Ltd.
 * https://tapit.vn
 *
 ******************************SD Card**********************************
 *  Description: Process files on SD Card
 *
 *  Version:  	1.0
 *  Author: 	Hoang Van Binh
 *  Release: 	Sep 27, 2021
 *  Built with STM32CubeIDE version 1.7.0
 *******************************************************************************
 */

#ifndef INC_SD_H_
#define INC_SD_H_

/* Includes ------------------------------------------------------------------*/
#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#include "fatfs_sd.h"
#include <stdlib.h>
#include "stm32f3xx_hal.h"
/* Defines ------------------------------------------------------------------*/
#define DEBUG_INFO huart2

/* Exported functions ---------------------------------------------------------*/

void print(char *string);

uint32_t SD_GetTotalSpace(void);

uint32_t SD_GetFreeSpace(void);

FRESULT SD_Mount(const TCHAR* path);

FRESULT SD_UnMount(const TCHAR* path);

FRESULT SD_Scan(char* Path);

FRESULT SD_CreateFile(char *name);

FRESULT SD_WriteFile(char *name, char * data, uint32_t size);

FRESULT SD_ReadFile(char *name, uint8_t *pData, uint32_t startPos, uint32_t size);

FRESULT SD_UpdateFile(char *name, char *data, uint32_t size);

FRESULT SD_DeleteFile(char *name);

FRESULT SD_Rename(const TCHAR* pathOld, const TCHAR* pathNew);

uint32_t SD_GetFileSize(char *name);

FRESULT SD_CreateDir(char *name);

#endif /* INC_SD_H_ */
