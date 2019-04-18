#include <stdio.h>

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
    /*0x58*/ f32 slideVelX;
    /*0x5C*/ f32 slideVelZ;
    /*0x9C*/ struct Controller *controller;
};


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

    m->slideVelX = m->vel[0];
    m->slideVelZ = m->vel[2];
}

static s32 act_flying(struct MarioState *m)
{
    update_flying(m);

    m->pos[1] += m->vel[1];

    // Change to TRUE to simulate null wall collisions (ceiling/oob/corner)
    if (FALSE) {
        m->faceAngle[0] -= 0x200;
        if (m->faceAngle[0] < -0x2AAA)
            m->faceAngle[0] = -0x2AAA;
    }

    return FALSE;
}


static void run(struct MarioState *m) {
    s32 frame = 0;

    m->pos[1] = 1000.0f;

    while (m->pos[1] > 0) {
        act_flying(m);

        frame += 1;
        printf("Frame %d: y = %f, v = %f\n", frame, m->pos[1], m->forwardVel);
    }
}

int main(void) {
    struct MarioState m = {};
    struct Controller controller = {};
    m.controller = &controller;
    run(&m);
}
