
#include "stm32h7xx_hal.h"

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

// select GPIO using 'A', 'B', etc
void GPIO_SetBit(char gpio, int pin, int state);

// select spi instance using integer
int SPI_SendData(char spi, int data_p, int size);

int SPI_SendDataDMA(char spi, int data_p, int size, void (*callback) (void));