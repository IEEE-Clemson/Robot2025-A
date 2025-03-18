#include "util.h"

float clamp(float x, float min_val, float max_val)
{
    if(x > max_val)
        return max_val;
    else if(x < min_val)
        return min_val;
    else
        return x;
}

float max(float x, float y) {
    if(x > y)
        return x;
    else
        return y;
}