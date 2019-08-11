#include "render.h"
#include "stm32h743xx.h"
#include "frequency_sensor.h"
#include "color.h"
#include "hsluv.h"
#include <arm_math.h>
#include <stdlib.h>

RenderMode2_t* NewRender2(Render2Params_t *params, ColorParams_t *colorParams, int size, int length,
    void (*setPixel) (int x, int y, Color_ABGR c)) {

    RenderMode2_t *r = (RenderMode2_t*)malloc(sizeof(RenderMode2_t));
    r->params = params;
    r->colorParams = colorParams;
    r->size = size;
    r->length = length;
    r->_setPixel = setPixel;

    return r;
}

static float sigmoid(float x) {
    float a = x;
    if (x < 0) a = -a;
    return (1.0 + x / (1.0 + a)) / 2.0;
}

static Color_ABGRf get_hsv(ColorParams_t *params, float amp, float phase, float phi) {
    float vs = params->valueScale;
    float vo = params->valueOffset;
    float ss = params->saturationScale;
    float so = params->saturationOffset;
    float as = params->alphaScale;
    float ao = params->alphaOffset;

    float hue = fmod(180 * (phi + phase) / PI, 360);
    if (hue < 0) hue += 360;

    // float sat = sigmoid(ss * amp + so);
    float val = ss * sigmoid(vs * amp + vo) + so;
    float alp = sigmoid(as * amp + ao);

    double r, g, b;
    hsluv2rgb((double)hue, 100, 100*(double)val, &r, &g, &b);
    r *= r;
    g *= g;
    b *= b;
    
    Color_ABGRf c = {alp * params->maxAlpha, (float)b, (float)g, (float)r};
    return c;
}

void Render2(RenderMode2_t *r, FS_Drivers_t *drivers) {
    int width = r->size / r->length;
    float spacing = (float)width / (float)(drivers->size + 1);

    Color_ABGRf frame[r->size];
    for (int i = 0; i < r->size; i++) {
        Color_ABGRf c = {0,0,0,0};
        frame[i] = c;
    }
    float *amp = FS_GetColumn(drivers, 0);
    float *phase = drivers->energy;
    float *scales = drivers->scales;

    float ph = r->params->pHeight;
    float phs = r->params->pHScale;
    float pho = r->params->pHOffset;
    float pvho = r->params->pVHOffset;

    float pw = r->params->pWidth;
    float pws = r->params->pWScale;
    float pwo = r->params->pWOffset;
    float pvwo = r->params->pVWOffset;

    for (int i = 0; i < drivers->size; i++) {
        float center = (i+1) * spacing;
        int y = i % r->length;
        int yoffset = y * width;

        float val = amp[i] * scales[i];
        float phi = 2*PI / r->colorParams->period * center;
        Color_ABGRf color = get_hsv(r->colorParams, val, phase[i], phi);

        float pheight = ph * sigmoid(phs * val + pvho) + pho;
        float pwidth = pw * sigmoid(pws * val + pvwo) + pwo;

        int start = (int)ceilf(center - pwidth);
        if (start < 0) start = 0;
        int end = (int)floorf(center + pwidth);
        if (end >= width) end = width - 1;

        for (int x = start; x < end; x++) {
            float xc = (x - center) / pwidth;
            float p = pheight * (1 - xc*xc);

            int idx = x + yoffset;
            frame[idx].a += p * color.a;
            frame[idx].b += p * color.b;
            frame[idx].g += p * color.g;
            frame[idx].r += p * color.r;
        }
    }

    

    for (int y = 0; y < r->length; y++) {
        int yoffset = y * width;
        for (int x = 0; x < width; x++) {
            int i = x + yoffset;
            int a = (int)(255 * frame[i].a);
            int b = (int)(255 * frame[i].b);
            int g = (int)(255 * frame[i].g);
            int d = (int)(255 * frame[i].r);
            if (a < 0) a = 0;
            if (a > 255) a = 255;
            if (b < 0) b = 0;
            if (b > 255) b = 255;
            if (g < 0) g = 0;
            if (g > 255) g = 255;
            if (d < 0) d = 0;
            if (d > 255) d = 255;

            Color_ABGR c = {a, b, g, d};

            r->_setPixel(x, y, c);
        }
    }
}
