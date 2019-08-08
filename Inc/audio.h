#ifndef __AUDIO_H
#define __AUDIO_H

#include "stm32h7xx_hal.h"
#include "stm32h743xx.h"

#define AUDIO_BUFFER_SIZE 2048
#define AUDIO_FRAME_SIZE 128

HAL_StatusTypeDef Audio_Init(SAI_HandleTypeDef *hsai);

#endif

