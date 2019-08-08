#include "audio.h"
#include "stm32h743xx.h"
#include "main.h"

uint32_t raw_audio_buffer[AUDIO_BUFFER_SIZE] __attribute__ ((section(".i2s_dma_buffer"))) __attribute__ ((aligned (32)));

volatile int audio_buffer_offset = 0;
static void audioM0Complete(DMA_HandleTypeDef *hdma);
static void audioM1Complete(DMA_HandleTypeDef *hdma);
static void audioError(DMA_HandleTypeDef *hdma);

HAL_StatusTypeDef Audio_Init(SAI_HandleTypeDef *hsai) {

  uint32_t *buffer0 = raw_audio_buffer;
  uint32_t *buffer1 = raw_audio_buffer + AUDIO_BUFFER_SIZE / 2;

  if (hsai->State == HAL_SAI_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsai);

    // hsai->pBuffPtr = pData;
    hsai->XferSize = AUDIO_BUFFER_SIZE;
    hsai->XferCount = AUDIO_BUFFER_SIZE;
    hsai->ErrorCode = HAL_SAI_ERROR_NONE;
    hsai->State = HAL_SAI_STATE_BUSY_RX;

 
    hsai->hdmarx->XferCpltCallback = audioM0Complete;
    hsai->hdmarx->XferM1CpltCallback = audioM1Complete;

    hsai->hdmarx->XferErrorCallback = audioError;
    hsai->hdmarx->XferAbortCallback = NULL;

    /* Enable the Rx DMA Stream */
    if (HAL_DMAEx_MultiBufferStart_IT(hsai->hdmarx, (uint32_t)&hsai->Instance->DR,
        (uint32_t)buffer0, (uint32_t)buffer1, hsai->XferSize) != HAL_OK) {
        
        __HAL_UNLOCK(hsai);
        return  HAL_ERROR; 
    }

    /* Check if the SAI is already enabled */
    if ((hsai->Instance->CR1 & SAI_xCR1_SAIEN) == 0U)
    {
      /* Enable SAI peripheral */
      __HAL_SAI_ENABLE(hsai);
    }

    /* Enable the interrupts for error handling */
    hsai->Instance->IMR |= SAI_IT_OVRUDR|SAI_IT_WCKCFG;
    // __HAL_SAI_ENABLE_IT(hsai, SAI_InterruptFlag(hsai, SAI_MODE_DMA));

    /* Enable SAI Rx DMA Request */
    hsai->Instance->CR1 |= SAI_xCR1_DMAEN;

    /* Process Unlocked */
    __HAL_UNLOCK(hsai);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

static void audioM0Complete(DMA_HandleTypeDef *hdma) {
    int next_offset = audio_buffer_offset + 2*AUDIO_FRAME_SIZE;
    next_offset %= AUDIO_BUFFER_SIZE;
    
    HAL_DMAEx_ChangeMemory(hdma, (uint32_t)(raw_audio_buffer+next_offset), MEMORY0);

    audio_buffer_offset += AUDIO_FRAME_SIZE;
    audio_buffer_offset %= AUDIO_BUFFER_SIZE;
}

static void audioM1Complete(DMA_HandleTypeDef *hdma) {
    int next_offset = audio_buffer_offset + 2*AUDIO_FRAME_SIZE;
    next_offset %= AUDIO_BUFFER_SIZE;
    
    HAL_DMAEx_ChangeMemory(hdma, (uint32_t)(raw_audio_buffer+next_offset), MEMORY1);

    audio_buffer_offset += AUDIO_FRAME_SIZE;
    audio_buffer_offset %= AUDIO_BUFFER_SIZE;
}

static HAL_StatusTypeDef SAI_Disable(SAI_HandleTypeDef *hsai)
{
  register uint32_t count = 4U * (SystemCoreClock / 7U / 1000U);
  HAL_StatusTypeDef status = HAL_OK;

  /* Disable the SAI instance */
  __HAL_SAI_DISABLE(hsai);

  do
  {
    /* Check for the Timeout */
    if (count == 0U)
    {
      /* Update error code */
      hsai->ErrorCode |= HAL_SAI_ERROR_TIMEOUT;
      status = HAL_TIMEOUT;
      break;
    }
    count--;
  }
  while ((hsai->Instance->CR1 & SAI_xCR1_SAIEN) != 0U);

  return status;
}

static void audioError(DMA_HandleTypeDef *hdma)
{
  SAI_HandleTypeDef *hsai = (SAI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Ignore DMA FIFO error */
  if (HAL_DMA_GetError(hdma) != HAL_DMA_ERROR_FE)
  {
    /* Set SAI error code */
    hsai->ErrorCode |= HAL_SAI_ERROR_DMA;

    /* Disable the SAI DMA request */
    hsai->Instance->CR1 &= ~SAI_xCR1_DMAEN;

    /* Disable SAI peripheral */
    /* No need to check return value because state will be updated and HAL_SAI_ErrorCallback will be called later */
    (void) SAI_Disable(hsai);

    /* Set the SAI state ready to be able to start again the process */
    hsai->State = HAL_SAI_STATE_READY;

    /* Initialize XferCount */
    hsai->XferCount = 0U;

    Error_Handler();
  }
}
