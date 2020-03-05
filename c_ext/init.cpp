#include <cstdio>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "math_util.h"
#include "flight.h"

namespace py = pybind11;
using namespace std;


static void clear_mario_state(struct MarioState *m) {
    struct Controller *c = m->controller;
    memset(c, 0, sizeof(*c));
    memset(m, 0, sizeof(*m));
    m->controller = c;
}

static void copy_mario_state(struct MarioState *dst, struct MarioState *src) {
    memcpy(dst, src, sizeof(*src));
}


static void act_flying_pitch_only(struct MarioState *m) {
    s16 targetPitchVel = -(s16) (m->controller->stickY * (m->forwardVel / 5.0f));

    if (targetPitchVel > 0) {
        if (m->angleVel[0] < 0) {
            m->angleVel[0] += 0x40;
            if (m->angleVel[0] > 0x20)
                m->angleVel[0] = 0x20;
        }
        else {
            m->angleVel[0] = approach_s32(m->angleVel[0], targetPitchVel, 0x20, 0x40);
        }
    }
    else if (targetPitchVel < 0) {
        if (m->angleVel[0] > 0) {
            m->angleVel[0] -= 0x40;
            if (m->angleVel[0] < -0x20)
                m->angleVel[0] = -0x20;
        }
        else {
            m->angleVel[0] = approach_s32(m->angleVel[0], targetPitchVel, 0x40, 0x20);
        }
    }
    else {
        m->angleVel[0] = approach_s32(m->angleVel[0], 0, 0x40, 0x40);
    }

    m->forwardVel -= 2.0f * ((f32) m->faceAngle[0] / 0x4000) + 0.1f;

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

    m->pos[1] += m->forwardVel * sins(m->faceAngle[0]);

    m->faceAngle[0] -= 0x200;
    if (m->faceAngle[0] < -0x2AAA)
        m->faceAngle[0] = -0x2AAA;
}


static f32 compute_max_height(struct MarioState *m, const vector<f32> &controls, s32 cap_time, f32 death_barrier) {
  f32 max_height = m->pos[1];
  for (size_t i = 0; i < controls.size(); i++) {
    if (i >= cap_time) {
      break;
    }
    m->controller->stickY = controls[i];
    act_flying(m, true);
    if (m->pos[1] < death_barrier) {
      break;
    }
    if (m->pos[1] > max_height) {
      max_height = m->pos[1];
    }
  }
  return max_height;
}


static f32 approx_max_height(struct MarioState *m, s32 frames, s32 granularity, f32 death_barrier) {
  // TODO: Iterative version

  if (frames <= 0 || m->pos[1] < death_barrier) {
    return m->pos[1];
  }

  f32 max_height = m->pos[1];

  struct MarioState m1 = *m;
  m1.controller->stickY = 64;
  for (s32 i = 0; i < granularity; i++) {
    act_flying(&m1, true);
    max_height = max(max_height, m1.pos[1]);
  }
  max_height = max(max_height, approx_max_height(&m1, frames - granularity, granularity, death_barrier));

  struct MarioState m2 = *m; // TODO: Don't need to copy
  m2.controller->stickY = -64;
  for (s32 i = 0; i < granularity; i++) {
    act_flying(&m2, true);
    max_height = max(max_height, m2.pos[1]);
  }
  max_height = max(max_height, approx_max_height(&m2, frames - granularity, granularity, death_barrier));

  // m->controller->stickY = -64;
  // for (s32 i = 0; i < granularity; i++) {
  //   act_flying(m, true);
  //   max_height = max(max_height, m->pos[1]);
  // }
  // max_height = max(max_height, approx_max_height(m, frames - granularity, granularity, death_barrier));

  return max_height;
}


static vector<f32> get_features(struct MarioState *m, s32 frames_left) {
  vector<f32> features;
  features.push_back(m->faceAngle[0]);
  features.push_back(m->angleVel[0]);
  features.push_back(m->pos[1]);
  features.push_back(m->forwardVel);
  features.push_back(1);
  return features;
}

static vector<f32> get_heights_from_weights(vector<f32> weights, s32 frames) {
  struct MarioState m = {};
  struct Controller c = {};
  m.controller = &c;

  m.pos[1] = 2000;
  m.forwardVel = 50;

  vector<f32> heights;
  heights.push_back(m.pos[1]);

  for (s32 i = 0; i < frames; i++) {
    vector<f32> features = get_features(&m, frames - i);

    f32 stick_y = 0;
    for (s32 i = 0; i < weights.size(); i++) {
      stick_y += features[i] * weights[i];
    }
    stick_y = min(max(stick_y, -64), 64);

    m.controller->stickY = stick_y;
    act_flying(&m, true);

    if (m.pos[1] < 0) {
      break;
    }
    heights.push_back(m.pos[1]);
  }

  return heights;
}


template<typename T>
class Vec3Wrapper {
private:
  T *data;
public:
  Vec3Wrapper(T *data) : data(data) {}

  T get_x() { return data[0]; }
  void set_x(T t) { data[0] = t; }
  T get_y() { return data[1]; }
  void set_y(T t) { data[1] = t; }
  T get_z() { return data[2]; }
  void set_z(T t) { data[2] = t; }
};


extern void optimize_for_stick_y(
  const MarioState &initial_m,
  size_t num_frames,
  function<void(const vector<f32> &)> callback
);


PYBIND11_MODULE(c_ext, m) {
  py::class_<Controller>(m, "Controller")
    .def(py::init<>())
    .def_readwrite("stick_x", &Controller::stickX)
    .def_readwrite("stick_y", &Controller::stickY);
  py::class_<Vec3Wrapper<f32>>(m, "Vec3f")
    .def_property("x", &Vec3Wrapper<f32>::get_x, &Vec3Wrapper<f32>::set_x)
    .def_property("y", &Vec3Wrapper<f32>::get_y, &Vec3Wrapper<f32>::set_y)
    .def_property("z", &Vec3Wrapper<f32>::get_z, &Vec3Wrapper<f32>::set_z);
  py::class_<Vec3Wrapper<s16>>(m, "Vec3s")
    .def_property("x", &Vec3Wrapper<s16>::get_x, &Vec3Wrapper<s16>::set_x)
    .def_property("y", &Vec3Wrapper<s16>::get_y, &Vec3Wrapper<s16>::set_y)
    .def_property("z", &Vec3Wrapper<s16>::get_z, &Vec3Wrapper<s16>::set_z);
  py::class_<MarioState>(m, "MarioState")
    .def(py::init<>())
    .def_property_readonly("face_angle", [](const MarioState &m) { return Vec3Wrapper<s16>((s16 *) m.faceAngle); })
    .def_property_readonly("angle_vel", [](const MarioState &m) { return Vec3Wrapper<s16>((s16 *) m.angleVel); })
    .def_property_readonly("pos", [](const MarioState &m) { return Vec3Wrapper<f32>((f32 *) m.pos); })
    // .def_property_readonly("vel", [](const MarioState &m) { return Vec3Wrapper<f32>((f32 *) m.vel); })
    .def_readwrite("forward_vel", &MarioState::forwardVel)
    .def_readwrite("controller", &MarioState::controller);
  m.def("clear_mario_state", clear_mario_state);
  m.def("copy_mario_state", copy_mario_state);
  m.def("adjust_analog_stick", adjust_analog_stick);
  m.def("act_flying", act_flying);
  m.def("compute_max_height", compute_max_height);
  m.def("approx_max_height", approx_max_height);
  m.def("get_features", get_features);
  m.def("get_heights_from_weights", get_heights_from_weights);
  m.def("optimize_for_stick_y", optimize_for_stick_y);
}
