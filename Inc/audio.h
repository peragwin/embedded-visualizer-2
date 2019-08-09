#ifndef __AUDIO_H
#define __AUDIO_H

#include "stm32h7xx_hal.h"

#define AUDIO_BUFFER_SIZE 2048
#define AUDIO_FRAME_SIZE 128
#define AUDIO_CHANNEL 1
#define AUDIO_FFT_SIZE (AUDIO_BUFFER_SIZE / 4)

HAL_StatusTypeDef Audio_Init(SAI_HandleTypeDef *hsai, uint32_t *audio_buffer);

#endif
