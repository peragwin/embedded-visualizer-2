#include "init.h"
#include "main.h"
// #include "ssd1331.h"
#include "audio.h"

extern SAI_HandleTypeDef hsai_BlockA1;

int main_init(void) {
    // SSD1331_Init();

    if (Audio_Init(&hsai_BlockA1) != HAL_OK) {
        Error_Handler();
    }

    return 0;
}