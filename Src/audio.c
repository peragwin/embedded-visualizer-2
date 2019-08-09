#include "audio.h"
#include "stm32h743xx.h"
#include "main.h"

extern MDMA_HandleTypeDef hmdma_mdma_channel40_sw_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel41_sw_0;

uint32_t raw_audio_buffer[AUDIO_BUFFER_SIZE] __attribute__ ((section(".i2s_dma_buffer"))) __attribute__ ((aligned (32)));
uint32_t *audio_frame;

volatile int audio_buffer_offset = 0;
volatile int mdma0InProgress = 0;
volatile int mdma1InProgress = 0;
volatile int audio_frame_ready = 0; // TODO use hw semaphore with interrupt?
static void audioM0Complete(DMA_HandleTypeDef *hdma);
static void audioM1Complete(DMA_HandleTypeDef *hdma);
static void audioError(DMA_HandleTypeDef *hdma);
static void copyBufferMDMA(int offset);
static void mdmaComplete(MDMA_HandleTypeDef *hmdma);
static void mdmaError(MDMA_HandleTypeDef *hmdma);

HAL_StatusTypeDef Audio_Init(SAI_HandleTypeDef *hsai, uint32_t *audio_buffer) {

  uint32_t *buffer0 = raw_audio_buffer;
  uint32_t *buffer1 = raw_audio_buffer + AUDIO_BUFFER_SIZE / 2;
  audio_frame = audio_buffer;

  HAL_NVIC_SetPriority(MDMA_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

  if (HAL_MDMA_RegisterCallback(&hmdma_mdma_channel40_sw_0, HAL_MDMA_XFER_CPLT_CB_ID,
    mdmaComplete) != HAL_OK) {
    
    return HAL_ERROR;
  }
  if (HAL_MDMA_RegisterCallback(&hmdma_mdma_channel40_sw_0, HAL_MDMA_XFER_ERROR_CB_ID,
    mdmaError) != HAL_OK) {
    
    return HAL_ERROR;
  }
  if (HAL_MDMA_RegisterCallback(&hmdma_mdma_channel41_sw_0, HAL_MDMA_XFER_CPLT_CB_ID,
    mdmaComplete) != HAL_OK) {
    
    return HAL_ERROR;
  }
  if (HAL_MDMA_RegisterCallback(&hmdma_mdma_channel41_sw_0, HAL_MDMA_XFER_ERROR_CB_ID,
    mdmaError) != HAL_OK) {
    
    return HAL_ERROR;
  }

  if (hsai->State == HAL_SAI_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsai);

    // hsai->pBuffPtr = pData;
    hsai->XferSize = AUDIO_FRAME_SIZE;
    // hsai->XferCount = AUDIO_BUFFER_SIZE;
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

    copyBufferMDMA(audio_buffer_offset);
}

void copyBufferMDMA(int offset) {
  int start = offset - (AUDIO_BUFFER_SIZE / 2);

  int wrap = start < 0;
  if (wrap) {
    int len = -start/2;
    start += AUDIO_BUFFER_SIZE;

    mdma0InProgress = 1;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    HAL_StatusTypeDef err = HAL_MDMA_Start_IT(&hmdma_mdma_channel40_sw_0,
      raw_audio_buffer+start+AUDIO_CHANNEL, audio_frame, len*4, 1);
    if (err != HAL_OK) {
      if (err != HAL_BUSY) {
        Error_Handler();
      } 
    }

    if (offset > 0) {
      mdma1InProgress = 1;
      err = HAL_MDMA_Start_IT(&hmdma_mdma_channel41_sw_0,
        raw_audio_buffer+AUDIO_CHANNEL, audio_frame+len, (offset/2)*4, 1);
      if (err != HAL_OK) {
        if (err != HAL_BUSY) {
          Error_Handler();
        } 
      }
    }

  } else {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    mdma0InProgress = 1;
    HAL_StatusTypeDef err = HAL_MDMA_Start_IT(&hmdma_mdma_channel40_sw_0,
      raw_audio_buffer+start+AUDIO_CHANNEL, audio_frame, (AUDIO_BUFFER_SIZE / 4) * 4, 1);
    if (err != HAL_OK) {
      if (err != HAL_BUSY) {
        Error_Handler();
      } 
    }
  }
}

static void mdmaComplete(MDMA_HandleTypeDef *hmdma) {
  if (hmdma->Instance == hmdma_mdma_channel40_sw_0.Instance) {
    mdma0InProgress = 0;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  } else {
    mdma1InProgress = 0;
  }
  if (!(mdma0InProgress || mdma1InProgress)) {
    audio_frame_ready = 1;
  }
}

static void mdmaError(MDMA_HandleTypeDef *hmdma) {
  Error_Handler();
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
