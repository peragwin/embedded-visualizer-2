#ifndef __RENDER_H
#define __RENDER_H

#include "frequency_sensor.h"
#include "color.h"

typedef struct {
    float valueScale;
    float valueOffset;
    float saturationScale;
    float saturationOffset;
    float alphaScale;
    float alphaOffset;
    float maxAlpha;
    ColorGamut_t gamut;
} ColorParams_t;

typedef struct {
    float pHeight;
    float pHScale;
    float pHOffset;
    float pVHOffset;
    float pWidth;
    float pWScale;
    float pWOffset;
    float pVWOffset;
} Render2Params_t;

// RenderMode2 displays a single row of LEDs
typedef struct {
    int size;
    ColorParams_t *colorParams;
    Render2Params_t *params;
    void (*_setPixel) (int x, Color_ABGR c);
} RenderMode2_t;

RenderMode2_t* NewRender2(Render2Params_t *params, ColorParams_t *colorParams, int size,
    void (*setPixel) (int x, Color_ABGR c));
void Render2(RenderMode2_t *r, FS_Drivers_t *drivers);

#endif