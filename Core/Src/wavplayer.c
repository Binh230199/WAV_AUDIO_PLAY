/*
 ******************************Audio Process*********************************
 *  Description: Process wav audio file
 *
 *  Version:  	1.0
 *  Author: 	Hoang Van Binh
 *  Release: 	Sep 22, 2021
 *  Built with STM32CubeIDE version 1.7.0
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "wavplayer.h"
#include "stm32f3xx_hal.h"
#include "string.h"

/* Variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac1_ch1;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Set sample frequency of audio file
 * @param  frequency: sample frequency
 * 			This parameter can be one of the following values:
 * 			+ 8000Hz
 * 			+ 16000Hz
 * 			+ 48000Hz
 * @retval None
 */
static void __WAV_SetSampleFreq(uint32_t frequency)
{
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 0;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = (uint16_t) ((F_HCLK / frequency) - 1);
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  Initialize the audio state
 * @param  wavInfo: Wave audio current info structure
 * @retval None
 */
void WAV_Init(WAV_InfoTypeDef* wavInfo)
{
	wavInfo->state = WAV_STATE_PLAY;
}

/**
 * @brief  Check the file is wav audio file?
 * @param  header: header of wav audio file
 * @retval '0' if the file is not wav format, '1' if it's correct
 */
static uint8_t __WAV_IsWavFile(WAV_FormatTypeDef *header)
{
	//compare chunkID == RIFF ?
	if(strncmp((char*) header->chunkID, "RIFF", 4) != 0)
		return 0;
	if(header->audioFormat != WAVE_FORMAT_PCM)
		return 0;
	return 1;
}

/**
 * @brief  Get audio statement
 * @param  wavInfo: Wave audio current info structure
 * @retval WAV state
 */
WAV_StateTypeDef WAV_GetState(WAV_InfoTypeDef* wavInfo)
{
	return wavInfo->state;
}

/**
 * @brief  Filter all file same extension in a direction
 * @param  path: directory want to filter
 * @param  extension: extension of the file
 * @param  fileList: FILELIST structure
 * @retval res
 */
FRESULT WAV_FilterFile(char* path, char* extension, FILELIST_FileTypeDef* fileList)
{
	FRESULT res = FR_OK;
	static FILINFO fno;
	DIR dir;
	char *f_name;
	fileList->ptr = 0;

	res = f_opendir(&dir, path);
	if(res == FR_OK)
	{
		while(1)
		{
			res = f_readdir(&dir, &fno);
			if(res != FR_OK || fno.fname[0] == 0) break;
			if(fno.fname[0] == '.')
				continue;
			f_name = fno.fname;
			if((fno.fattrib & AM_DIR) == 0) // it is a file
			{
				if(strstr(f_name, extension))
				{
					strcpy((char*)fileList->flName[fileList->ptr], (char*)f_name);
					fileList->ptr++;
				}
				else __NOP();
			}
			else __NOP();
		}
		print("Files were found: \r\n");
		for(int i = 0; i < fileList->ptr; i++)
		{
			char *string = malloc(15*sizeof(char));
			sprintf(string, "%d. %s\n", i+1, fileList->flName[i]);
			print(string);
			free(string);
		}
	}
	else __NOP();
	res = f_closedir(&dir);
	fileList->nbFile = fileList->ptr;
	return res;
}

/**
 * @brief  Play wave audio file, this function needs to set WAV_STATE_PLAY before play
 * @param  fileList: FILELIST structure
 * @param  wavInfo: Wave audio current info structure
 * @retval WAV state
 */
WAV_StateTypeDef WAV_Play(char* nameFile, WAV_InfoTypeDef* wavInfo)
{
	static FRESULT res;
	static UINT br;
	static FIL fil;
	static uint32_t f_size = 0;
	static uint16_t bufSize = 0;
	WAV_FormatTypeDef header;
	wavInfo->file = nameFile;

	if(WAV_GetState(wavInfo) != WAV_STATE_PLAY)
		goto end;
	res = f_open(&fil, wavInfo->file, FA_READ);
	if(res != FR_OK)
	{
		wavInfo->state = WAV_STATE_ERROR;
	}
	else
	{
		res = f_read(&fil, &header, 44, &br);
		if(res != FR_OK)
		{
			wavInfo->state = WAV_STATE_ERROR;
		}
		else
		{
			if(__WAV_IsWavFile(&header))
			{
				f_size = header.fileSize - 36;
				__WAV_SetSampleFreq(header.sampleRate);
				HAL_TIM_Base_Start(&htim6);
				memset(wavInfo->buff, 0, WAV_BUFFER_SIZE);

				while(f_size > 0)
				{
					if(f_size < WAV_BUFFER_SIZE) bufSize = f_size;
					else bufSize = WAV_BUFFER_SIZE;

					f_lseek(&fil, wavInfo->filePtr);
					f_read(&fil, wavInfo->buff, bufSize, &br);

					HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t*)wavInfo->buff, bufSize, DAC_ALIGN_8B_R);

					while(HAL_DAC_GetState(&hdac1) != HAL_DAC_STATE_READY)
					{
						if(WAV_GetState(wavInfo) != WAV_STATE_PLAY)
							goto end;
					}
					f_size -= bufSize;
					wavInfo->filePtr += bufSize;
				}
			}
			else __NOP();
		}
	}
	wavInfo->filePtr = 0;
	f_lseek(&fil, 0);

	end:
	f_close(&fil);
	HAL_TIM_Base_Stop(&htim6);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	return wavInfo->state;
}

/**
 * @brief  Pause wave audio file
 * @param  fileList: FILELIST structure
 * @param  wavInfo: Wave audio current info structure
 * @retval WAV state
 */
WAV_StateTypeDef WAV_Pause(WAV_InfoTypeDef* wavInfo)
{
	wavInfo->state = WAV_STATE_PAUSE;
	HAL_TIM_Base_Stop(&htim6);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	return wavInfo->state;
}

/**
 * @brief  Resume wave audio file
 * @param  fileList: FILELIST structure
 * @param  wavInfo: Wave audio current info structure
 * @retval WAV state
 */
WAV_StateTypeDef WAV_Resume(WAV_InfoTypeDef* wavInfo)
{
//	static FIL fil;
	//	if(f_open(&fil, (const TCHAR*) wavInfo->file, FA_READ) != FR_OK)
	//		return wavInfo->state = WAV_STATE_ERROR;
	//	else
//	f_open(&fil, (const TCHAR*) wavInfo->file, FA_READ);
	wavInfo->state = WAV_STATE_PLAY;
	HAL_TIM_Base_Stop(&htim6);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	return wavInfo->state;
}

/**
 * @brief  Stop wave audio file
 * @param  fileList: FILELIST structure
 * @param  wavInfo: Wave audio current info structure
 * @retval WAV state
 */
WAV_StateTypeDef WAV_Stop(WAV_InfoTypeDef* wavInfo)
{
	static FIL fil;
	wavInfo->state = WAV_STATE_STOP;
	wavInfo->filePtr = 0;
	f_lseek(&fil, 0);
	HAL_TIM_Base_Stop(&htim6);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	return wavInfo->state;
}

/**
 * @brief  Next wave audio file
 * @param  fileList: FILELIST structure
 * @param  wavInfo: Wave audio current info structure
 * @retval WAV state
 */
WAV_StateTypeDef WAV_Next(FILELIST_FileTypeDef* fileList, WAV_InfoTypeDef* wavInfo)
{
	wavInfo->number++;
	wavInfo->state = WAV_STATE_NEXT;
	if(wavInfo->number > fileList->nbFile)
		wavInfo->number = 0;
	return wavInfo->state;
}

/**
 * @brief  Back wave audio file
 * @param  fileList: FILELIST structure
 * @param  wavInfo: Wave audio current info structure
 * @retval WAV state
 */
WAV_StateTypeDef WAV_Back(FILELIST_FileTypeDef* fileList, WAV_InfoTypeDef* wavInfo)
{
	wavInfo->number--;
	wavInfo->state = WAV_STATE_PREVIOUS;
	if(wavInfo->number < 0)
		wavInfo->number = fileList->nbFile;
	return wavInfo->state;
}
