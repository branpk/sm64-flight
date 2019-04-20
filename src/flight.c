#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "math_util.h"


#define PRINTF_HEX(x) ((x) < 0 ? "-" : ""), ((x) < 0 ? -(x) : (x))


static FILE *tasInputs;


struct Controller
{
  /*0x04*/ float stickX;        // [-64, 64] positive is right
  /*0x08*/ float stickY;        // [-64, 64] positive is up
};

struct MarioState
{
    /*0x2C*/ Vec3s faceAngle;
    /*0x32*/ Vec3s angleVel;
    /*0x3C*/ Vec3f pos;
    /*0x48*/ Vec3f vel;
    /*0x54*/ f32 forwardVel;
    /*0x9C*/ struct Controller *controller;

    s16 movePitch;
};

static void clear_mario_state(struct MarioState *m) {
    struct Controller *c = m->controller;
    memset(c, 0, sizeof(*c));
    memset(m, 0, sizeof(*m));
    m->controller = c;
}

static void adjust_analog_stick(struct Controller *controller, s16 rawStickX, s16 rawStickY)
{
    if (rawStickX < -128 || rawStickX > 127 || rawStickY < -128 || rawStickY > 127) {
        printf("Bad raw stick: %d %d\n", rawStickX, rawStickY);
        exit(1);
    }

    // reset the controller's x and y floats.
    controller->stickX = 0;
    controller->stickY = 0;

    // modulate the rawStickX and rawStickY to be the new float values by adding/subtracting 6.
    if(rawStickX <= -8)
        controller->stickX = rawStickX + 6;

    if(rawStickX >=  8)
        controller->stickX = rawStickX - 6;

    if(rawStickY <= -8)
        controller->stickY = rawStickY + 6;

    if(rawStickY >=  8)
        controller->stickY = rawStickY - 6;

    // calculate float magnitude from the center by vector length.
    f32 stickMag = sqrtf(controller->stickX * controller->stickX
                               + controller->stickY * controller->stickY);

    // magnitude cannot exceed 64.0f: if it does, modify the values appropriately to
    // flatten the values down to the allowed maximum value.
    if(stickMag > 64)
    {
        controller->stickX  *= 64 / stickMag;
        controller->stickY  *= 64 / stickMag;
        stickMag = 64;
    }
}



static void update_flying_yaw(struct MarioState *m)
{
    s16 targetYawVel = -(s16) (m->controller->stickX * (m->forwardVel / 4.0f));

    if (targetYawVel > 0)
    {
        if (m->angleVel[1] < 0)
        {
            m->angleVel[1] += 0x40;
            if (m->angleVel[1] > 0x10)
                m->angleVel[1] = 0x10;
        }
        else
        {
            m->angleVel[1] = approach_s32(m->angleVel[1], targetYawVel, 0x10, 0x20);
        }
    }
    else if (targetYawVel < 0)
    {
        if (m->angleVel[1] > 0)
        {
            m->angleVel[1] -= 0x40;
            if (m->angleVel[1] < -0x10)
                m->angleVel[1] = -0x10;
        }
        else
        {
            m->angleVel[1] = approach_s32(m->angleVel[1], targetYawVel, 0x20, 0x10);
        }
    }
    else
    {
        m->angleVel[1] = approach_s32(m->angleVel[1], 0, 0x40, 0x40);
    }

    m->faceAngle[1] += m->angleVel[1];
    m->faceAngle[2] = 20 * -m->angleVel[1];
}

static void update_flying_pitch(struct MarioState *m)
{
    s16 targetPitchVel = -(s16) (m->controller->stickY * (m->forwardVel / 5.0f));

    if (targetPitchVel > 0)
    {
        if (m->angleVel[0] < 0)
        {
            m->angleVel[0] += 0x40;
            if (m->angleVel[0] > 0x20)
                m->angleVel[0] = 0x20;
        }
        else
        {
            m->angleVel[0] = approach_s32(m->angleVel[0], targetPitchVel, 0x20, 0x40);
        }
    }
    else if (targetPitchVel < 0)
    {
        if (m->angleVel[0] > 0)
        {
            m->angleVel[0] -= 0x40;
            if (m->angleVel[0] < -0x20)
                m->angleVel[0] = -0x20;
        }
        else
        {
            m->angleVel[0] = approach_s32(m->angleVel[0], targetPitchVel, 0x40, 0x20);
        }
    }
    else
    {
        m->angleVel[0] = approach_s32(m->angleVel[0], 0, 0x40, 0x40);
    }
}

static void update_flying(struct MarioState *m)
{
    update_flying_pitch(m);
    update_flying_yaw(m);

    m->forwardVel -= 2.0f * ((f32) m->faceAngle[0] / 0x4000) + 0.1f;
    m->forwardVel -= 0.5f * (1.0f - coss(m->angleVel[1]));

    if (m->forwardVel < 0.0f)
        m->forwardVel = 0.0f;

    if (m->forwardVel > 16.0f)
        m->faceAngle[0] += (m->forwardVel - 32.0f) * 6.0f;
    else if (m->forwardVel > 4.0f)
        m->faceAngle[0] += (m->forwardVel - 32.0f) * 10.0f;
    else
        m->faceAngle[0] -= 0x400;

    m->faceAngle[0] += m->angleVel[0];

    if (m->faceAngle[0] > 0x2AAA)
        m->faceAngle[0] = 0x2AAA;
    if (m->faceAngle[0] < -0x2AAA)
        m->faceAngle[0] = -0x2AAA;

    m->vel[0] = m->forwardVel * coss(m->faceAngle[0]) * sins(m->faceAngle[1]);
    m->vel[1] = m->forwardVel * sins(m->faceAngle[0]);
    m->vel[2] = m->forwardVel * coss(m->faceAngle[0]) * coss(m->faceAngle[1]);
}

static s32 act_flying(struct MarioState *m, s32 downTilt)
{
    if (m->controller->stickY < -64 || m->controller->stickY > 64) {
        printf("Invalid stickY = %f\n", m->controller->stickY);
        exit(1);
    }

    update_flying(m);

    m->pos[1] += m->vel[1];

    if (downTilt) {
        m->faceAngle[0] -= 0x200;
        if (m->faceAngle[0] < -0x2AAA)
            m->faceAngle[0] = -0x2AAA;
    }

    return FALSE;
}


static void update_flying_controlled(struct MarioState *m, s16 movementPitch, s32 downTilt)
{
    // update_flying_pitch(m);
    // update_flying_yaw(m);

    m->forwardVel -= 2.0f * ((f32) m->faceAngle[0] / 0x4000) + 0.1f;
    // m->forwardVel -= 0.5f * (1.0f - coss(m->angleVel[1]));

    if (m->forwardVel < 0.0f)
        m->forwardVel = 0.0f;

    // if (m->forwardVel > 16.0f)
    //     m->faceAngle[0] += (m->forwardVel - 32.0f) * 6.0f;
    // else if (m->forwardVel > 4.0f)
    //     m->faceAngle[0] += (m->forwardVel - 32.0f) * 10.0f;
    // else
    //     m->faceAngle[0] -= 0x400;

    // m->faceAngle[0] += m->angleVel[0];
    m->faceAngle[0] = movementPitch; // Assume full control of m->angleVel[0]

    if (m->faceAngle[0] > 0x2AAA)
        m->faceAngle[0] = 0x2AAA;
    if (m->faceAngle[0] < -0x2AAA)
        m->faceAngle[0] = -0x2AAA;

    m->vel[0] = m->forwardVel * coss(m->faceAngle[0]) * sins(m->faceAngle[1]);
    m->vel[1] = m->forwardVel * sins(m->faceAngle[0]);
    m->vel[2] = m->forwardVel * coss(m->faceAngle[0]) * coss(m->faceAngle[1]);
}

static s32 act_flying_controlled(struct MarioState *m, s16 movementPitch, s32 downTilt)
{
    update_flying_controlled(m, movementPitch, downTilt);

    m->pos[1] += m->vel[1];

    if (downTilt) {
        m->faceAngle[0] -= 0x200;
        if (m->faceAngle[0] < -0x2AAA)
            m->faceAngle[0] = -0x2AAA;
    }

    return FALSE;
}


static void update_flying_no_control(struct MarioState *m)
{
    m->forwardVel -= 2.0f * ((f32) m->faceAngle[0] / 0x4000) + 0.1f;
    m->forwardVel -= 0.5f * (1.0f - coss(m->angleVel[1]));

    if (m->forwardVel < 0.0f)
        m->forwardVel = 0.0f;

    if (m->forwardVel > 16.0f)
        m->faceAngle[0] += (m->forwardVel - 32.0f) * 6.0f;
    else if (m->forwardVel > 4.0f)
        m->faceAngle[0] += (m->forwardVel - 32.0f) * 10.0f;
    else
        m->faceAngle[0] -= 0x400;

    m->faceAngle[0] += m->angleVel[0];

    if (m->faceAngle[0] > 0x2AAA)
        m->faceAngle[0] = 0x2AAA;
    if (m->faceAngle[0] < -0x2AAA)
        m->faceAngle[0] = -0x2AAA;

    m->movePitch = m->faceAngle[0];

    m->vel[0] = m->forwardVel * coss(m->faceAngle[0]) * sins(m->faceAngle[1]);
    m->vel[1] = m->forwardVel * sins(m->faceAngle[0]);
    m->vel[2] = m->forwardVel * coss(m->faceAngle[0]) * coss(m->faceAngle[1]);
}

static s32 act_flying_no_control(struct MarioState *m, s32 downTilt)
{
    update_flying_no_control(m);

    m->pos[1] += m->vel[1];

    if (downTilt) {
        m->faceAngle[0] -= 0x200;
        if (m->faceAngle[0] < -0x2AAA)
            m->faceAngle[0] = -0x2AAA;
    }

    return FALSE;
}

static s32 pitch_offset_for_move_pitch(struct MarioState *m, s16 movePitch) {
    s16 pitch = m->faceAngle[0];
    if (m->forwardVel > 16.0f)
        pitch += (m->forwardVel - 32.0f) * 6.0f;
    else if (m->forwardVel > 4.0f)
        pitch += (m->forwardVel - 32.0f) * 10.0f;
    else
        pitch -= 0x400;

    return movePitch - pitch;
}

static s32 pitch_vel_for_pitch_offset_pos(s32 offset) {
    f32 n = (-1.0f + sqrtf(1.0f + (8.0f * offset) / 0x40)) / 2.0f;
    return (s32)(n * 0x40);
}

static s32 pitch_vel_for_pitch_offset(s32 offset) {
    if (offset >= 0) {
        return pitch_vel_for_pitch_offset_pos(offset);
    } else {
        return -pitch_vel_for_pitch_offset_pos(-offset);
    }
}

static s32 min_pitch_vel_disp(struct MarioState *m, s32 pitchVel) {
    f32 speed = m->forwardVel;
    s32 disp = 0;

    while (pitchVel != 0) {
        if (m->forwardVel > 16.0f)
            disp += (speed - 32.0f) * 6.0f;
        else if (m->forwardVel > 4.0f)
            disp += (speed - 32.0f) * 10.0f;
        else
            disp -= 0x400;

        disp += pitchVel;

        disp -= 0x200;

        pitchVel = approach_s32(pitchVel, 0, 0x40, 0x40);
    }

    return disp;
}

static f32 pitch_vel_for_pitch(struct MarioState *m, s32 targetPitch) {
    f32 bestPitchVel = 0;
    s32 minDist = 100000;

    for (f32 pv = -0x400; pv < 0x400; pv += 1) {
        f32 disp = min_pitch_vel_disp(m, pv);
        f32 dist = abs((targetPitch - m->faceAngle[0]) - disp);

        if (dist < minDist) {
            minDist = dist;
            bestPitchVel = pv;
        }
    }

    return bestPitchVel;
}

static f32 max_possible_min_y(struct MarioState *m) {
    f32 y = m->pos[1];
    f32 speed = m->forwardVel;
    s32 pitch = m->faceAngle[0];
    s32 pitchVel = m->angleVel[0];

    while (TRUE) {
        pitchVel += 0x40;

        speed -= 2.0f * ((f32) pitch / 0x4000) + 0.1f;

        if (speed > 16.0f)
            pitch += (speed - 32.0f) * 6.0f;
        else if (speed > 4.0f)
            pitch += (speed - 32.0f) * 10.0f;
        else
            pitch -= 0x400;

        pitch += pitchVel;

        if (pitch >= 0) {
            break;
        }

        y += speed * sins(pitch);

        pitch -= 0x200;
    }

    return y;
}

static s16 constrain_target_pitch_vel(struct MarioState *m, s16 targetPitchVel) {
    s16 maxv = (s16)(64.0f * (m->forwardVel / 5.0f));
    s16 minv = (s16)(-64.0f * (m->forwardVel / 5.0f));
    return min(max(targetPitchVel, minv), maxv);
}

static s16 approach_pitch_vel(s16 pitchVel, s16 targetPitchVel)
{
    if (targetPitchVel > 0)
    {
        if (pitchVel < 0)
        {
            pitchVel += 0x40;
            if (pitchVel > 0x20)
                pitchVel = 0x20;
        }
        else
        {
            pitchVel = approach_s32(pitchVel, targetPitchVel, 0x20, 0x40);
        }
    }
    else if (targetPitchVel < 0)
    {
        if (pitchVel > 0)
        {
            pitchVel -= 0x40;
            if (pitchVel < -0x20)
                pitchVel = -0x20;
        }
        else
        {
            pitchVel = approach_s32(pitchVel, targetPitchVel, 0x40, 0x20);
        }
    }
    else
    {
        pitchVel = approach_s32(pitchVel, 0, 0x40, 0x40);
    }
    return pitchVel;
}


static f32 raw_stick_to_stick_y(s16 rawStickX, s16 rawStickY)
{
    f32 stickX;
    f32 stickY;

    if (rawStickX < -128 || rawStickX > 127 || rawStickY < -128 || rawStickY > 127) {
        printf("Bad raw stick: %d %d\n", rawStickX, rawStickY);
        exit(1);
    }

    // reset the controller's x and y floats.
    stickX = 0;
    stickY = 0;

    // modulate the rawStickX and rawStickY to be the new float values by adding/subtracting 6.
    if(rawStickX <= -8)
        stickX = rawStickX + 6;

    if(rawStickX >=  8)
        stickX = rawStickX - 6;

    if(rawStickY <= -8)
        stickY = rawStickY + 6;

    if(rawStickY >=  8)
        stickY = rawStickY - 6;

    // calculate float magnitude from the center by vector length.
    f32 stickMag = sqrtf(stickX * stickX + stickY * stickY);

    // magnitude cannot exceed 64.0f: if it does, modify the values appropriately to
    // flatten the values down to the allowed maximum value.
    if(stickMag > 64)
    {
        stickX  *= 64 / stickMag;
        stickY  *= 64 / stickMag;
        stickMag = 64;
    }

    return stickY;
}

static s16 approach_pitch_vel_raw_stick_y(struct MarioState *m, f32 targetPitchVel) {
    s16 bestRawStickY;
    s32 closestDist = 1000000;

    for (int rawStickY = -128; rawStickY < 128; rawStickY++) {
        f32 stickY = raw_stick_to_stick_y(0, rawStickY);
        s16 actualPitchVel = -(s16) (stickY * (m->forwardVel / 5.0f));
        s32 dist = abs(targetPitchVel - actualPitchVel);

        if (dist < closestDist) {
            bestRawStickY = rawStickY;
            closestDist = dist;
        }
    }

    // f32 stickY = -(f32)targetPitchVel * 5.0f / m->forwardVel;
    // stickY = min(max(stickY, -64.0f), 64.0f);

    return bestRawStickY;
}

// static f32 approach_pitch_vel_stick_y(struct MarioState *m, s16 targetPitchVel) {
//     f32 stickY = -(f32)targetPitchVel * 5.0f / m->forwardVel;
//     return min(max(stickY, -64.0f), 64.0f);
// }

// static s16 stick_y_to_raw_stick_y(f32 stickY) {
//     s16 rawStickY = (s16)((stickY / 64) * 128);
//     return min(max(rawStickY, -128), 127);
// }


static f32 energy(struct MarioState *m) {
    return m->forwardVel * m->forwardVel + 4.0f / 3.141592653f * m->pos[1];
}

static s32 speed_jerk(struct MarioState *m, f32 speed) {
    if (speed > 16.0f)
        return (speed - 32.0f) * 6.0f;
    else if (speed > 4.0f)
        return (speed - 32.0f) * 10.0f;
    else
        return -0x400;
}

static f32 total_speed_jerk(struct MarioState *m, f32 speed) {
    return speed_jerk(m, speed) - 0x200;
}

static s32 halting_pitch(struct MarioState *m) {
    s32 pitch = 0;
    s32 vel = m->angleVel[0];
    f32 jerk = total_speed_jerk(m, m->forwardVel);

    while (vel != 0) {
        vel = approach_s32(vel, 0, 0x40, 0x40);
        pitch += vel + jerk;
    }

    return pitch;
}

static void target_pitch(struct MarioState *m, s16 targetPitch) {
    // printf("%d, %d, %d\n", m->faceAngle[0], halting_pitch(m), targetPitch);
    if (m->faceAngle[0] + halting_pitch(m) < targetPitch) {
        m->controller->stickY = -64;
    } else {
        m->controller->stickY = 64;
    }
}


// static void run(struct MarioState *m) {
//     clear_mario_state(m);
// }


// static void run(struct MarioState *m) {
//     s32 frame = 0;

//     m->pos[1] = 4000.0f;
//     m->forwardVel = 30.0f;

//     s32 phase = -1;

//     f32 maxY = -1000000.0f;
//     f32 maxV = -1000000.0f;

//     while (1) {
//         if (phase < 0) {
//             m->controller->stickY = 64;
//             if (m->pos[1] < 0) {
//                 phase = 1;
//                 // printf("(v) Frame %d: y = %f, v = %f\n", frame, m->pos[1], m->forwardVel);
//             }
//         } else {
//             // target_pitch(m, 0x2000);
//             m->controller->stickY = -32;
//             if (m->pos[1] > 0) {
//                 phase = -1;
//                 // printf("(^) Frame %d: y = %f, v = %f\n", frame, m->pos[1], m->forwardVel);
//             }
//         }

//         act_flying(m);

//         frame += 1;

//         s32 print = FALSE;
//         if (m->pos[1] > maxY) {
//             maxY = m->pos[1];
//             print = TRUE;
//         }
//         if (m->forwardVel > maxV) {
//             maxV = m->forwardVel;
//             print = TRUE;
//         }
//         if (print) {
//             printf("Frame %d: y = %f, v = %f\n", frame, m->pos[1], m->forwardVel);
//         }
//     }
// }

// // INSTANT PITCH CHANGE
// static void run(struct MarioState *m) {
//     s32 frame = 0;

//     m->pos[1] = 0.0f;
//     m->forwardVel = 50.0f;

//     int phase = 1;
//     s16 movePitch = 0;
//     s16 maxChange = 0x200;

//     while (frame < 10000) {
//         if (phase == 1) {
//             movePitch = approach_s32(movePitch, 0x11B0, maxChange, maxChange);
//             act_flying_controlled(m, movePitch, TRUE);
//             if (m->forwardVel < 20.0f) {
//                 phase = -1;
//                 printf("Frame %d: y = %f, v = %f\n", frame, m->pos[1], m->forwardVel);
//             }
//         } else {
//             movePitch = approach_s32(movePitch, -0x2AAA, maxChange, maxChange);
//             act_flying_controlled(m, movePitch, TRUE);
//             if (m->pos[1] < -7500.0f) {
//                 phase = 1;
//                 // printf("Frame %d: y = %f, v = %f\n", frame, m->pos[1], m->forwardVel);
//             }
//         }

//         frame += 1;
//         // printf("Frame %d: y = %f, v = %f\n", frame, m->pos[1], m->forwardVel);
//     }
// }

// In video: 21 min for y = 5629
// Best: 3.93 minutes

static f32 run(struct MarioState *m) {
    s32 frame = 0;

    // First: 2279

    // *(u32 *)&m->pos[1] = 0xC4C1F742;
    // *(u32 *)&m->forwardVel = 0x42C7CD92;
    // m->faceAngle[0] = -10922;
    // m->angleVel[0] =  ;

    // m->pos[1] = 0.0f;
    // m->forwardVel = 100.0f;

    // m->pos[1] -= 500;

    s32 phase = -1;
    // s16 movePitch = 0;
    // s16 pitchVel = 0;
    // s16 pitchAcc = 0x20;
    // s16 maxVel = 0x100;
    f32 minY = 1000000;
    f32 maxY = -1000000;
    f32 lastMaxY = maxY;

    f32 startY = m->pos[1];
    s16 rawStickY;

    s32 totalFrames = -1;
    s16 maxPitch = 0;
    s32 printEachFrame = FALSE;

    f32 initialY = m->pos[1];
    f32 initialV = m->forwardVel;
    s32 initialP = m->faceAngle[0];
    s32 initialPV = m->angleVel[0];

    // printf("%f\n", 2648 - startY);

    // while (TRUE) {
    while (frame < 15000) {
        f32 targetPitchVel;
        if (phase == 1) {
            // s32 targetOffset = pitch_offset_for_move_pitch(m, 0x1280);
            // targetPitchVel = pitch_vel_for_pitch_offset(targetOffset);
            targetPitchVel = pitch_vel_for_pitch(m, 0x1200);
            // if (m->angleVel[0] > 0x280) {
            //     targetPitchVel = 0;
            // }
            // targetPitchVel = max(min(targetPitchVel, 0x204), -0x200);
            rawStickY = approach_pitch_vel_raw_stick_y(m, targetPitchVel);

            adjust_analog_stick(m->controller, 0, rawStickY);
            act_flying(m, TRUE);

            if (m->forwardVel < 30.0f) {
                phase = -1;
                // m->angleVel[0] = 0;
                // printf("Frame %d: y = %f, v = %f, miny = %f, maxy = %f, maxp: %s0x%X\n", frame, m->pos[1], m->forwardVel, minY, maxY, PRINTF_HEX(maxPitch));
                maxPitch = 0;

                printf("%s Frame %d: y = %f, v = %f, miny = %f, maxy = %f, dmaxy = %f\n", phase < 0 ? "v" : "^", frame, m->pos[1], m->forwardVel, minY, maxY, maxY - lastMaxY);
                lastMaxY = maxY;
                // minY = 100000;

                if (frame > 39700) {
                    printEachFrame = TRUE;
                }
            }
        } else {
            // s32 targetOffset = pitch_offset_for_move_pitch(m, -0x2AAA + 0x200);
            // targetPitchVel = pitch_vel_for_pitch_offset(targetOffset);
            targetPitchVel = pitch_vel_for_pitch(m, -0x2AAA);
            rawStickY = approach_pitch_vel_raw_stick_y(m, targetPitchVel);

            adjust_analog_stick(m->controller, 0, rawStickY);
            act_flying(m, TRUE);

            // TODO: Play with -2500 for higher sequences
            // if (m->forwardVel > 160) {
            // if (max_possible_min_y_after_down(m) < -6000) {
            s32 threshold = 3500;
            if ((maxY < threshold && m->pos[1] - startY < max(maxY - startY - 4200.0f, 0) - 2500.0f) ||
                (maxY >= threshold && m->pos[1] < -3400)) {
                phase = 1;
                // m->angleVel[0] = 0;
                // printf("%s Frame %d: y = %f, v = %f, miny = %f, maxy = %f\n", phase < 0 ? "v" : "^", frame, m->pos[1], m->forwardVel, minY, maxY);
                maxPitch = 0;
            }
        }

        frame += 1;
        if (printEachFrame) {
            printf("%s Frame %d: sy = %d, y = %f, v = %f, p = %s0x%X, pv = %s0x%X, tpv = %s0x%X\n",
                phase < 0 ? "v" : "^",
                frame,
                rawStickY,
                m->pos[1],
                m->forwardVel,
                PRINTF_HEX(m->faceAngle[0]),
                PRINTF_HEX(m->angleVel[0]),
                PRINTF_HEX((s16)targetPitchVel));
        }
        fprintf(tasInputs, "0000 00%02x ", (u8)rawStickY);

        if (m->pos[1] < minY) {
            minY = m->pos[1];
        }
        if (m->pos[1] > maxY) {
            maxY = m->pos[1];
        }
        if (m->faceAngle[0] > maxPitch) {
            maxPitch = m->faceAngle[0];
        }

        if (maxY >= 5629 && totalFrames < 0) {
            totalFrames = frame;
        }
    }

    printf("\nInitial state:\n");
    printf("pos y = %f\n", initialY);
    printf("h speed = %f\n", initialV);
    printf("pitch = %d\n", initialP);
    printf("pitch vel = %d\n", initialPV);

    printf("\nSimulated 60 seconds\n");

    if (minY < -8191 + 2048) {
        printf("Died (initial state might be too low. if you really need this, let me know and I might be able to make it work)\n");
    } else {
        printf("max y = %f\n", maxY);
        printf("Wrote outputs to tas_inputs.txt\n");
    }

    if (totalFrames >= 0) {
        printf("\nMinutes to 5629: %f\n", (f32)totalFrames / 30 / 60);
    }

    return maxY;
}

s64
strtol64(const char *nptr, char **endptr, register int base)
{
    register const char *s = nptr;
    register u64 acc;
    register int c;
    register u64 cutoff;
    register int neg = 0, any, cutlim;

    /*
     * Skip white space and pick up leading +/- sign if any.
     * If base is 0, allow 0x for hex and 0 for octal, else
     * assume decimal; if base is already 16, allow 0x.
     */
    do {
        c = *s++;
    } while (c == ' ');
    if (c == '-') {
        neg = 1;
        c = *s++;
    } else if (c == '+')
        c = *s++;
    if ((base == 0 || base == 16) &&
        c == '0' && (*s == 'x' || *s == 'X')) {
        c = s[1];
        s += 2;
        base = 16;
    }
    if (base == 0)
        base = c == '0' ? 8 : 10;

    /*
     * Compute the cutoff value between legal numbers and illegal
     * numbers.  That is the largest legal value, divided by the
     * base.  An input number that is greater than this value, if
     * followed by a legal input character, is too big.  One that
     * is equal to this value may be valid or not; the limit
     * between valid and invalid numbers is then based on the last
     * digit.  For instance, if the range for longs is
     * [-2147483648..2147483647] and the input base is 10,
     * cutoff will be set to 214748364 and cutlim to either
     * 7 (neg==0) or 8 (neg==1), meaning that if we have accumulated
     * a value > 214748364, or equal but the next digit is > 7 (or 8),
     * the number is too big, and we will return a range error.
     *
     * Set any if any `digits' consumed; make it negative to indicate
     * overflow.
     */
    // cutoff = neg ? -(u64)LONG_MIN : LONG_MAX;
    // cutlim = cutoff % (u64)base;
    // cutoff /= (u64)base;
    for (acc = 0, any = 0;; c = *s++) {
        if (c >= '0' && c <= '9')
            c -= '0';
        else if (c >= 'a' && c <= 'z' || c >= 'A' && c <= 'Z')
            c -= (c >= 'A' && c <= 'Z') ? 'A' - 10 : 'a' - 10;
        else
            break;
        if (c >= base)
            break;
        // if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim)) {}
            // any = -1;
        // }
        // else {
            any = 1;
            acc *= base;
            acc += c;
        // }?
    }
    if (any < 0) {
        // acc = neg ? LONG_MIN : LONG_MAX;
        // errno = ERANGE;
    } else if (neg)
        acc = -acc;
    if (endptr != 0)
        *endptr = (char *) (any ? s - 1 : nptr);
    return (acc);
}

int main(int argc, char **argv) {
    if (argc < 5) {
        printf("usage: flight.exe <posy in hex> <hspeed in hex> <pitch> <pitch vel>\n");
        exit(1);
    }

    u32 y = strtol64(argv[1], NULL, 0);
    u32 v = strtol64(argv[2], NULL, 0);
    s32 p = strtol64(argv[3], NULL, 0);
    s32 pv = strtol64(argv[4], NULL, 0);

    tasInputs = fopen("tas_inputs.txt", "w");

    // Maximize height for speed loss:
    // f32 maxv = 0;
    // for (int p = 0; p < 0x10000; p += 0x10) {
    //     f32 v = sins(p) / (2.0f * ((f32)(p - 0x200) / 0x4000) + 0.1);
    //     if (v > maxv) {
    //         printf("%X -> %f\n", p, v);
    //         maxv = v;
    //     }
    // }

    struct MarioState m = {};
    struct Controller controller = {};
    m.controller = &controller;

    m.pos[1] = *(f32 *)&y;
    m.forwardVel = *(f32 *)&v;
    m.faceAngle[0] = p;
    m.angleVel[0] = pv;

    run(&m);
}


// Notes:
// For maximizing height gained vs speed lost, use movement pitch 0x11B0
// (This won't actually maximize total height gained from a given speed - that
// is more complicated I think)
// Without 0x200 tilt, it's 0x17F0
