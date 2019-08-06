#ifndef __ROT_H
#define __ROT_H

typedef struct rot {
    void (*increment)(void);
    void (*decrement)(void);
} Rotary;

extern Rotary rot0;
extern Rotary rot1;

void ROT_Increment(Rotary *r);
void ROT_Decrement(Rotary *r);

#endif