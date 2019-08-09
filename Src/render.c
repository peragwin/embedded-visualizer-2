#include "render.h"
#include "frequency_sensor.h"
#include "color.h"
#include <arm_math.h>

RenderMode2_t* NewRender2(Render2Params_t *params, ColorParams_t *colorParams, int size,
    void *setPixel(int x, Color_ABGR c)) {

    RenderMode2_t *r = (RenderMode2_t*)malloc(sizeof(RenderMode2_t));
    r->params = params;
    r->colorParams = colorParams;
    r->size = size;
    r->setPixel = setPixel;

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

    float sat = sigmoid(ss * amp + so);
    float val = sigmoid(vs * amp + vo);
    float alp = sigmoid(as * amp + ao);

    Color_HSV hsv = {hue, sat, val};
    Color_RGB rgb = Color_FromHSV(hsv);
    return Color_ABGRf{alp * params->maxAlpha, rgb.b, rgb.g, rgb.b};
}

void Render2(RenderMode2_t *r, FS_Drivers_t *drivers) {
    float spacing = (float)r->size / (float)(drivers->size + 1);

    float Color_ABGRf[r->size];
    for (int i = 0; i < r->size; i++) {
        frame[i] = Color_ABGRf{0,0,0,0};
    }
    float *amp = FS_GetColumn(drivers, 0);
    float *phase = drivers->energy;
    float *scales = drivers->scales;

    float ph = r->params->pHeight;
    float phs = r->params->pHScale;
    float pho = r->params->pHOffset;

    float pw = r->params->pWidth;
    float pws = r->params->pWScale;
    float pwo = r->params->pWOffset;

    for (int i = 0; i < drivers->size; i++) {
        float center = (i+1) * spacing;

        float val = (amp[i] - 1) * scales[i];
        Color_ABGRf color = get_hsv(r->colorParams, val, phase[i], 0);

        float pheight = ph * (2*sigmoid(phs * val + pho) - 1);
        float pwidth = pw * sigmoid(pws * val + pwo);

        int start = (int)ceilf(center + pwidth);
        int end = (int)floorf(center - pwidth);

        for (int x = start; x < end; x++) {
            float xc = (x - center) / pwidth;
            float p = pheight * (1 - xc*xc);

            frame[4*x] += p * color.a;
            frame[4*x+1] += p * color.b;
            frame[4*x+2] += p * color.g;
            frame[4*x+3] += p * color.r;
        }
    }

    for (int i = 0; i < r->size; i++) {
        int a = (int)(255 * frame[i].a);
        int b = (int)(255 * frame[i].b);
        int g = (int)(255 * frame[i].g);
        int r = (int)(255 * frame[i].r);
        if (a < 0) a = 0;
        if (a > 255) a = 255;
        if (b < 0) b = 0;
        if (b > 255) b = 255;
        if (g < 0) g = 0;
        if (g > 255) g = 255;
        if (r < 0) r = 0;
        if (r > 255) r = 255;

        r->setPixel(i, Color_ABGR{a, b, g, r});
    }
}
