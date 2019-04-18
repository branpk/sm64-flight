#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "math_util.h"


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
};

static void clear_mario_state(struct MarioState *m) {
    struct Controller *c = m->controller;
    memset(c, 0, sizeof(*c));
    memset(m, 0, sizeof(*m));
    m->controller = c;
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

static s32 act_flying(struct MarioState *m)
{
    update_flying(m);

    m->pos[1] += m->vel[1];

    if (FALSE) {
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

static void run(struct MarioState *m) {
    s32 frame = 0;

    m->pos[1] = 0.0f;
    m->forwardVel = 100.0f;

    int phase = -1;
    s16 movePitch = 0;
    s16 pitchVel = 0;
    s16 pitchAcc = 0x20;
    // s16 maxVel = 0x100;

    while (frame < 10000) {
        if (phase == 1) {
            f32 maxTargVel = max(64.0f * (m->forwardVel / 5.0f) - 0x200, 0);
            pitchVel = approach_s32(pitchVel, maxTargVel, pitchAcc, pitchAcc);
            movePitch = approach_s32(movePitch, 0x11B0, pitchVel, pitchVel);
            act_flying_controlled(m, movePitch, TRUE);
            if (m->forwardVel < 40.0f) {
                phase = -1;
                pitchVel = 0;
                printf("Frame %d: y = %f, v = %f\n", frame, m->pos[1], m->forwardVel);
            }
        } else {
            f32 maxTargVel = 64.0f * (m->forwardVel / 5.0f);
            pitchVel = approach_s32(pitchVel, maxTargVel, pitchAcc, pitchAcc);
            movePitch = approach_s32(movePitch, -0x2AAA - 0x200, pitchVel, pitchVel);
            act_flying_controlled(m, movePitch, TRUE);
            if (m->pos[1] < -2000.0f) {
                phase = 1;
                pitchVel = 0;
                // printf("Frame %d: y = %f, v = %f\n", frame, m->pos[1], m->forwardVel);
            }
        }

        frame += 1;
        // printf("Frame %d: y = %f, v = %f\n", frame, m->pos[1], m->forwardVel);
    }
}

int main(void) {
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
    run(&m);
}


// Notes:
// For maximizing height gained vs speed lost, use movement pitch 0x11B0
// (This won't actually maximize total height gained from a given speed - that
// is more complicated I think)
// Without 0x200 tilt, it's 0x17F0
