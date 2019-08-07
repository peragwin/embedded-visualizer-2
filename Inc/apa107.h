#ifndef __APA107_H
#define __APA107_H

#include "color.h"

#define APA107_MAX_ALPHA 31

#define APA107_BUFFER_DATA_SIZE(w,h) (4 * ((w)*(h)) + 1)
#define APA107_BUFFER_ENDFRAME_SIZE(w,h) (6 + ((w)*(h)/16))
#define APA107_BUFFER_SIZE(w,h) (APA107_BUFFER_DATA_SIZE(w,h) + APA107_BUFFER_ENDFRAME_SIZE(w,h))

// Define to enable cool transpose effect?
// #define APA107_TRANSPOSE

typedef struct apa107 {
    int width;
    int height;
    int _size;
    char *buffer;
    void (*_send) (char *buffer, int size);
} APA107;

APA107* APA107_Init(int w, int h, char *buffer, void (*send) (char *buffer, int size));
void APA107_SetPixel(APA107 *a, int x, int y, Color_ABGR c);
void APA107_Show(APA107 *a);

#endif
