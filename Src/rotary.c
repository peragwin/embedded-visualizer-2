#include "rotary.h"

Rotary rot0;
Rotary rot1;

void ROT_Increment(Rotary *r) {
    if (r->increment) r->increment();
}

void ROT_Decrement(Rotary *r) {
    if (r->decrement) r->decrement();
}
