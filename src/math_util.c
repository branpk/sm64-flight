#include "math_util.h"

#include "math_tables.h"

s32 approach_s32(s32 current, s32 target, s32 inc, s32 dec)
{
    //! If target is close to the max or min s32, then it's possible to overflow
    // past it without stopping.

    if (current < target)
    {
        current += inc;
        if (current > target)
            current = target;
    }
    else
    {
        current -= dec;
        if (current < target)
            current = target;
    }
    return current;
}

f32 approach_f32(f32 current, f32 target, f32 inc, f32 dec)
{
    if (current < target)
    {
        current += inc;
        if (current > target)
            current = target;
    }
    else
    {
        current -= dec;
        if (current < target)
            current = target;
    }
    return current;
}

static u16 atan2_lookup(f32 a, f32 b)
{
    u16 ret;

    if (b == 0)
        ret = D_8038B000[0];
    else
        ret = D_8038B000[(s32)(a / b * 1024 + 0.5f)];
    return ret;
}

/**
 * Note that the argument order is swapped from the standard atan2.
 */
s16 atan2s(f32 a, f32 b)
{
    u16 ret;

    if (b >= 0)
    {
        if (a >= 0)
        {
            if (a >= b)
                ret = atan2_lookup(b, a);
            else
                ret = 0x4000 - atan2_lookup(a, b);
        }
        else
        {
            a = -a;
            if (a < b)
                ret = 0x4000 + atan2_lookup(a, b);
            else
                ret = 0x8000 - atan2_lookup(b, a);
        }
    }
    else
    {
        b = -b;
        if (a < 0)
        {
            a = -a;
            if (a >= b)
                ret = 0x8000 + atan2_lookup(b, a);
            else
                ret = 0xC000 - atan2_lookup(a, b);
        }
        else
        {
            if (a < b)
                ret = 0xC000 + atan2_lookup(a, b);
            else
                ret = -atan2_lookup(b, a);
        }
    }
    return ret;
}
