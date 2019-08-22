#include "loop.h"
#include "ssd1331.h"
#include "stm32h7xx_hal.h"
#include "main.h"
#include "rotary.h"
#include "apa107.h"
#include "audio.h"
#include "frequency_sensor.h"
#include "render.h"
#include "color.h"

extern volatile int audio_frame_ready;
extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi1;
extern SAI_HandleTypeDef hsai_BlockA1;
extern TIM_HandleTypeDef htim6;
extern DAC_HandleTypeDef hdac1;

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

// uint8_t display_buffer[APA107_BUFFER_SIZE(DISPLAY_WIDTH, DISPLAY_HEIGHT)] __attribute__ ((section(".spi_dma_buffer"))) __attribute__ ((aligned (32)));
uint16_t display_buffer[SSD1331_DISPLAYWIDTH * SSD1331_DISPLAYHEIGHT] __attribute__ ((section(".spi_dma_buffer"))) __attribute__ ((aligned (32)));
uint16_t dacBuffer[AUDIO_FFT_SIZE] __attribute__ ((section(".dac_dma_buffer"))) __attribute__ ((aligned (32)));
float clutBuffer[360 * 100 * 3] __attribute__ ((section(".clut_buffer"))) __attribute__ ((aligned (32)));

// void txDisplayDMA(uint8_t *buffer, int size) {
//     // __HAL_SPI_CLEAR_EOTFLAG(&hspi1);
//     // __HAL_SPI_ENABLE_IT(&hspi1, SPI_IT_EOT);
//     HandleError(HAL_SPI_Transmit_DMA(&hspi1, (uint32_t)display_buffer, size), 1);
// }

// APA107 *display = NULL;
Audio_Processor_t *audio;
uint32_t audio_buffer[AUDIO_FFT_SIZE];
// RenderMode2_t *render;
RenderMode3_t *render;

// static void drawPixel(int x, int y, Color_ABGR c) {
//     // int y = 0;
//     // if (x >= DISPLAY_WIDTH) {
//     //     x -= DISPLAY_WIDTH;
//     //     y = 1;
//     // }
//     APA107_SetPixel(display, x, y, c);
// }

static void setPixel(int x, int y, Color_ABGR c) {
    SSD1331_SetPixel(display_buffer, x, y, c);
}

static void showDisplay(void) {
    SCB_CleanDCache_by_Addr(display_buffer, SSD1331_DISPLAYWIDTH*SSD1331_DISPLAYHEIGHT*2);
    SSD1331_ShowBufferDMA(display_buffer);
}

void main_loop(void) {
    // init_sin_table();
    rot0.increment = incrementRot0;
    rot0.decrement = decrementRot0;
    rot1.increment = incrementRot1;
    rot1.decrement = decrementRot1;

	// Color_RGB color;

	// LCD_Printf("ILI9486 driver\n"
	// 	"STM32F4 - NUCLEO-F446RE\n"
	// 	"Bare metal",
	// 	30, 80, color);

    if (Audio_Init(&hsai_BlockA1, audio_buffer) != HAL_OK) {
        Error_Handler();
    }

    audio = NewAudioProcessor(AUDIO_FFT_SIZE, 36, 60, dacBuffer);
    // display = APA107_Init(DISPLAY_WIDTH, DISPLAY_HEIGHT, display_buffer, txDisplayDMA);
    // Render2Params_t renderParams = {
    //     .pHeight = 1,
    //     .pHScale = 1,
    //     .pHOffset = -0.5,
    //     .pVHOffset = 1,
    //     .pWidth = 12,
    //     .pWScale = .15,
    //     .pWOffset = 0,
    //     .pVWOffset = -.5,
    // };
    Render3_Params_t renderParams = {
        .warpScale = 4,
        .warpOffset = .8,
        .scaleScale = 1,
        .scaleOffset = 1,
        .aspect = 1,
    };

    ColorParams_t colorParams = {
        .valueScale = 1,
        .valueOffset = 0,
        .saturationScale = .75,
        .saturationOffset = 0,
        .alphaScale = 0,
        .alphaOffset = 0,
        .maxAlpha = 0.5,
        .period = DISPLAY_WIDTH * 3,
        .gamut = {
            .red = 1,
            .green = 1,
            .blue = 1,
        },
        .clut = clutBuffer,
    };
    // render = NewRender2(&renderParams, &colorParams, DISPLAY_WIDTH*DISPLAY_HEIGHT, DISPLAY_HEIGHT, drawPixel);
    render = NewRender3(SSD1331_DISPLAYWIDTH, SSD1331_DISPLAYHEIGHT, 36, 60,
        &renderParams, &colorParams,
        setPixel,
        showDisplay
    );


    if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, audio->dacOutput, audio->size, DAC_ALIGN_12B_R) != HAL_OK) {
        Error_Handler();
    }

    uint16_t ph = 0;
    // uint16_t buffer[96*64];

    // for (int i = 0; i < 75; i++) { // 75 for 16
    //     if (i < 64 && i % 4 == 0){
    //         buffer[i] = 0xE0;
    //     }
    //     buffer[i] = 0;
    // }
    // buffer[68] = 0xFF;
    // buffer[8] = 0xFF;

    //     for (int i = 0; i < 64; i++) {
    //         color = Color16(i, i, i);
    //         ILI9486_DrawPixels(30 + i, 120, 1, 1, &color);
    //     }

    while (1) {
        //HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

        // // ph = rot0_value + 20*rot1_value;
        //ph++;
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

        if (audio_frame_ready) {
            audio_frame_ready = 0;

            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
            Audio_Process(audio, audio_buffer);
            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

            if (ph++ % 16 == 0) { 

            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            Render3(render, audio->fs->drivers);

            // SCB_CleanDCache_by_Addr(display_buffer, APA107_BUFFER_SIZE(DISPLAY_WIDTH, DISPLAY_HEIGHT));
            // APA107_Show(display);

            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            }
        }
    }
}

int toggle = 0;

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim1.Instance) {
        toggle++;
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

        // if (toggle ^= 1) {
        //      buffer[0] = 0xFF;
        //     buffer[1] = 0xFF;
        //     buffer[2] = 0xFF;
        //     buffer[3] = 0xFF;
        // } else {
        //     buffer[0] = 0xE0;
        //     buffer[1] = 0x00;
        //     buffer[2] = 0x00;
        //     buffer[3] = 0x00;
        // }


        // if (HAL_SPI_Transmit_DMA(&hspi1, (uint32_t)buffer, 14) != HAL_OK) {
        //     Error_Handler();
        // }

        // if (display == NULL) return;

        // Color_HSV c = {0, 1., 1.};
        // Color_RGB rgb;

        // for (int i = 0; i < DISPLAY_WIDTH; i++) {
        //     c.h = (toggle + i) % 360;
        //     rgb = Color_FromHSV(c);
        //     Color_ABGR abgr = {16, 255*rgb.b, 255*rgb.g, 255*rgb.r};
        //     APA107_SetPixel(display, i, 0, abgr);
        // }
        // // TODO: use MPU to disable cache on buffer
        // SCB_CleanDCache();
        // APA107_Show(display);
    }
}