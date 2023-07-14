/*
 * wavaudio.h
 *
 *  Created on: Sep 22, 2021
 *      Author: Administrator
 */

#ifndef INC_WAV_H_
#define INC_WAV_H_

/* Includes ----------------------------*/
#include "main.h"
#include "stdbool.h"
#include "stdio.h"

#define  AUDIO_BUFFER_SIZE 8000
//#define AUDIO_BUFFER_SIZE 4096

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
} WAVE_FormatTypeDef;


#endif /* INC_WAV_H_ */
