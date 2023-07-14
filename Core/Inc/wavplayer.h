/* --COPYRIGHT--,
 * Copyright (c) 2021, TAPIT Engineering Co., Ltd.
 * https://tapit.vn
 *
 ******************************Audio Process*********************************
 *  Description: Process wav audio file
 *
 *  Version:  	1.0
 *  Author: 	Hoang Van Binh
 *  Release: 	Sep 22, 2021
 *  Built with STM32CubeIDE version 1.7.0
 *******************************************************************************
 */

#ifndef INC_WAVPLAYER_H_
#define INC_WAVPLAYER_H_
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"
#include "stdio.h"
#include "stm32f3xx_hal.h"
#include "sd.h"

/* Defines ------------------------------------------------------------------*/
#define  F_HCLK 72000000 // define your HCLK clock frequency
#define  WAV_BUFFER_SIZE 2000

#define  CHUNK_ID                    0x52494646  /* correspond to the letters 'RIFF' */
#define  FILE_FORMAT                 0x57415645  /* correspond to the letters 'WAVE' */
#define  FORMAT_ID                   0x666D7420  /* correspond to the letters 'fmt ' */
#define  DATA_ID                     0x64617461  /* correspond to the letters 'data' */
#define  FACT_ID                     0x66616374  /* correspond to the letters 'fact' */
#define  WAVE_FORMAT_PCM             0x01
#define  FORMAT_CHUNK_SIZE           0x10
#define  CHANNEL_MONO                0x01
//#define  CHANNEL_STEREO              0x02
//#define  SAMPLE_RATE_8000            8000
#define  SAMPLE_RATE_16000           16000
//#define  SAMPLE_RATE_22050           22050
//#define  SAMPLE_RATE_44100           44100
#define  BITS_PER_SAMPLE_8           8
//#define  BITS_PER_SAMPLE_16        16


/* Exported types ------------------------------------------------------------*/
/* struct header of wav audio file */
typedef struct {
  char chunkID[4];       /* 0 */
  uint32_t fileSize;      /* 4 */
  uint32_t fileFormat;    /* 8 */

  uint32_t subChunk1ID;   /* 12 */
  uint32_t subChunk1Size; /* 16*/
  uint16_t audioFormat;   /* 20 */
  uint16_t numChannels;   /* 22 */
  uint32_t sampleRate;    /* 24 */
  uint32_t byteRate;      /* 28 */
  uint16_t blockAlign;    /* 32 */
  uint16_t bitPerSample;  /* 34 */

  char subChunk2ID[4];   /* 36 */
  uint32_t subChunk2Size; /* 40 */
} WAV_FormatTypeDef;

/* struct file list contains name of file and its number*/
typedef struct
{
	char flName[20][13];
	uint8_t ptr;
	uint8_t nbFile;
} FILELIST_FileTypeDef;

/*struct store current parameter of audio file*/
typedef struct
{
  uint8_t buff[WAV_BUFFER_SIZE];
  char* file;
  uint16_t number;
  uint32_t filePtr;
  uint32_t restSize;
  uint8_t state;
} WAV_InfoTypeDef;


typedef enum
{
  WAV_STATE_IDLE = 0,
  WAV_STATE_PLAY,
  WAV_STATE_STOP,
  WAV_STATE_PAUSE,
  WAV_STATE_RESUME,
  WAV_STATE_NEXT,
  WAV_STATE_PREVIOUS,
  WAV_STATE_ERROR,
} WAV_StateTypeDef;

/* Exported functions ---------------------------------------------------------*/

void WAV_Init(WAV_InfoTypeDef* wavInfo);

FRESULT WAV_FilterFile(char* path, char* extension, FILELIST_FileTypeDef* fileList);

WAV_StateTypeDef WAV_Play(char* nameFile, WAV_InfoTypeDef* wavInfo);

WAV_StateTypeDef WAV_Pause(WAV_InfoTypeDef* wavInfo);

WAV_StateTypeDef WAV_Resume(WAV_InfoTypeDef* wavInfo);

WAV_StateTypeDef WAV_Stop(WAV_InfoTypeDef* wavInfo);

WAV_StateTypeDef WAV_Next(FILELIST_FileTypeDef* fileList, WAV_InfoTypeDef* wavInfo);

WAV_StateTypeDef WAV_Back(FILELIST_FileTypeDef* fileList, WAV_InfoTypeDef* wavInfo);

WAV_StateTypeDef WAV_GetState(WAV_InfoTypeDef* wavInfo);

#endif /* INC_WAVPLAYER_H_ */
