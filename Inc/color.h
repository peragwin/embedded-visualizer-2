#ifndef __COLOR_H
#define __COLOR_H

typedef struct {
    float r;       // a fraction between 0 and 1
    float g;       // a fraction between 0 and 1
    float b;       // a fraction between 0 and 1
} Color_RGB;

typedef struct {
    float h;       // angle in degrees
    float s;       // a fraction between 0 and 1
    float v;       // a fraction between 0 and 1
} Color_HSV;

typedef struct {
    char a;
    char b;
    char g;
    char r;
} Color_ABGR;

Color_RGB Color_FromHSV(Color_HSV in);

#endif