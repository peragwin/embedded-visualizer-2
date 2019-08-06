#include "hal.h"

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
