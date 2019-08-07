#include "apa107.h"
#include <stdlib.h>

// APA107_Init returns a pointer to a new APA107. Since @buffer may need to be defined in
// special memory (ie non-TCM and non-cached for the DMA controller to have access), it
// is an optional arguement. If NULL, the buffer will be allocated.
APA107* APA107_Init(int w, int h, char *buffer, void (*send) (char *buffer, int size)) {
    APA107* display = (APA107*)malloc(sizeof(APA107));
    
    display->width = w;
    display->height = h;
    display->_send = send;
    display->_size = APA107_BUFFER_SIZE(w, h);
    
    if (buffer == NULL) {
        buffer = (char*)calloc(display->_size, sizeof(char));
    }
    display->buffer = buffer;

    // set the marker to start the end frame.
    buffer[APA107_BUFFER_DATA_SIZE(w, h)] = 0xFF;

    return display;
}

void setBuffer(APA107 *a, int idx, Color_ABGR c) {
    idx = 4 * (idx + 1);
    *((int*)(a->buffer+idx)) = 0xE0 | (int)*((int*)&c);
}

void APA107_SetPixel(APA107 *a, int x, int y, Color_ABGR c) {
    c.a >>= 3;

    // display grids are wired like a snake so every row needs to be flipped
    if (y%2 == 1) {
        x = a->width - 1 - x;
    }

    #ifdef APA107_TRANSPOSE
    int idx = x * a->height + y;
    #else
    int idx = y * a->width + x;
    #endif

    setBuffer(a, idx, c);
}

void APA107_Show(APA107 *a) {
    a->_send(a->buffer, a->_size);
}
