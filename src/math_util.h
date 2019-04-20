#ifndef MATH_UTIL_H_
#define MATH_UTIL_H_

#include <stdint.h>

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef float f32;
typedef double f64;

#define TRUE 1
#define FALSE 0

typedef f32 Vec2f[2];
typedef f32 Vec3f[3];
typedef s16 Vec3s[3];
typedef s32 Vec3i[3];
typedef f32 Vec4f[4];
typedef s16 Vec4s[4];

typedef f32 Mat4[4][4];

extern f32 gSineTable[];

#define sins(x) gSineTable[(u16) (x) >> 4]
#define coss(x) gSineTable[(u16) (x + 0x4000) >> 4]

#define min(a, b) ((a) <= (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define sqr(x) ((x) * (x))

#define abs(x) ((x) < 0 ? -(x) : (x))

s32 approach_s32(s32 a, s32 b, s32 c, s32 d);
f32 approach_f32(f32 a, f32 b, f32 c, f32 d);
s16 atan2s(f32 a, f32 b);

#endif
