#include "hal.h"
#include "main.h"

void GPIO_SetBit(char gpio, int pin, int state) {
    GPIO_TypeDef *g;
    switch (gpio) {
        case 'A':
            g = GPIOA;
            break;
        case 'B':
            g = GPIOB;
            break;
        case 'C':
            g = GPIOC;
            break;
        default:
            g = GPIOD;
        // TODO
    }
    GPIO_PinState s = GPIO_PIN_RESET;
    if (state) s = GPIO_PIN_SET;
    HAL_GPIO_WritePin(g, (uint32_t)(1<<pin), s);
}

int SPI_SendData(char spi, int data_p, int size) {
    SPI_HandleTypeDef *s;
    if (spi == 1) {
        s = &hspi1;
    } else {
        return -2;
    }
    if (HAL_SPI_Transmit(s, (uint32_t)data_p, size, 1000) != HAL_OK) {
        return -1;
    }
    return 0;
}

static void (*spi_callback) (void);

void SPI_DMA_Callback(DMA_HandleTypeDef *hdma) {
    spi_callback();
}

int SPI_SendDataDMA(char spi, int data_p, int size, void (*callback) (void)) {
    SPI_HandleTypeDef *s;
    if (spi == 1) {
        s = &hspi1;
    } else {
        return -2;
    }
    spi_callback = callback;
    s->hdmatx->XferCpltCallback = SPI_DMA_Callback;
    HandleError(HAL_SPI_Transmit_DMA(s, data_p, size), 0);
    return 0;
}
