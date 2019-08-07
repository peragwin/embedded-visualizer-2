#include "loop.h"
#include "ssd1331.h"
#include "stm32h7xx_hal.h"
#include "main.h"
#include "rotary.h"

extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi1;

float sin_table[256];

float sin_fast(float x) {
    x /= (2*3.14159);
    x *= 256;
    float xf = floorf(x);
    int i = (int)xf;
    int j = (int)ceilf(x);
    i %= 256;
    j %= 256;

    float l = sin_table[i];
    float r = sin_table[j];

    return l + (r - l) * (x - xf);
}

void init_sin_table() {
  for (int i = 0; i < 256; i++) {
    sin_table[i] = sin(2*3.14159/256*i);
  }
}

static int rot0_value = 0;
static int rot1_value = 125;

void setTim1Period(void) {
    uint16_t val = 20*rot1_value + rot0_value;
    htim1.Instance->ARR = val;
}

void incrementRot0(void) {
    rot0_value++;
    setTim1Period();
}
void decrementRot0(void) {
    rot0_value--;
    setTim1Period();
}
void incrementRot1(void) {
    rot1_value++;
    setTim1Period();
}
void decrementRot1(void) {
    rot1_value--;
    setTim1Period();
}

uint8_t red = 255;
uint8_t gre = 255;
uint8_t blu = 255;

uint8_t buffer[75];

void main_loop(void) {
    init_sin_table();
    rot0.increment = incrementRot0;
    rot0.decrement = decrementRot0;
    rot1.increment = incrementRot1;
    rot1.decrement = decrementRot1;

	uint16_t color = Color16(28, 45, 10);
	// LCD_Printf("ILI9486 driver\n"
	// 	"STM32F4 - NUCLEO-F446RE\n"
	// 	"Bare metal",
	// 	30, 80, color);

    uint16_t ph = 0;
    // uint16_t buffer[96*64];

    for (int i = 0; i < 75; i++) { // 75 for 16
        if (i < 64 && i % 4 == 0){
            buffer[i] = 0xE0;
        }
        buffer[i] = 0;
    }
    buffer[68] = 0xFF;
    // buffer[8] = 0xFF;

    //     for (int i = 0; i < 64; i++) {
    //         color = Color16(i, i, i);
    //         ILI9486_DrawPixels(30 + i, 120, 1, 1, &color);
    //     }
        

    while (1) {
        // HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

        // // ph = rot0_value + 20*rot1_value;
        // ph++;
        // color = Color16(red, gre, blu); 
        
        // for (int i = 0; i < 96*64; i++) {
        //     int x = i % 96;
        //     int y = i / 96;
        //     uint8_t r = (uint8_t)(126 * (sin_fast(2.0*3.14159/1024 * (float)(ph+x/2+y/4)) + 1));
        //     uint8_t g = (uint8_t)(126 * (sin_fast(2.0*3.14159/1024 * (float)(ph+x/2+y/4) + 2.0*3.14159/3) + 1));
        //     uint8_t b = (uint8_t)(126 * (sin_fast(2.0*3.14159/1024 * (float)(ph + x/2 + y/4) + 4.0*3.14159/3) + 1));
        //     color = Color16(r, g, b);
        //     uint16_t t = color >> 8;
        //     t &= 0xFF;
        //     t |= color << 8;
            
        //     // r = r >> 3;
        //     // uint8_t rev = (r >> 4) & 1;
        //     // rev |= (r >> 2) & 2;
        //     // rev |= r & 4;
        //     // rev |= (r << 2) & 8;
        //     // rev |= (r << 4) & 16;
        //     // buffer[i] = rev << 11;
            
        //     // color = 1 << ((i / 96) % 16);

        //     buffer[i] = t; //color;
        // }
        // SSD1331_DrawPixels(0, 0, 96, 64, (uint8_t*)buffer);
    }
}

int toggle = 0;

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim1.Instance) {
        // toggle++;
        // toggle %= 16;
        // int m1 = toggle - 1;
        // if (m1 < 0) m1 += 16;

       
        //     buffer[4*toggle] = 0xFF;
        //     buffer[4*toggle+1] = 0xFF;
        //     buffer[4*toggle+2] = 0x00;
        //     buffer[4*toggle+3] = 0xFF;
        
        //     buffer[4*m1] = 0xE0;
        //     buffer[4*m1+1] = 0x00;
        //     buffer[4*m1+2] = 0x00;
        //     buffer[4*m1+3] = 0x00;

        if (toggle ^= 1) {
             buffer[0] = 0xFF;
            buffer[1] = 0xFF;
            buffer[2] = 0xFF;
            buffer[3] = 0xFF;
        } else {
            buffer[0] = 0xE0;
            buffer[1] = 0x00;
            buffer[2] = 0x00;
            buffer[3] = 0x00;
        }


        if (HAL_SPI_Transmit_DMA(&hspi1, (uint32_t)buffer, 14) != HAL_OK) {
            Error_Handler();
        }
    }
}