#include <vector>
#include <map>
#include <optional>
#include <tuple>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <string>
#include <functional>
#include <random>
#include <ctime>
#include <memory>
#include <array>
#undef NDEBUG
#include <cassert>

#include "math_util.h"
#include "flight.h"

using namespace std;


// TODO: Use templates to make this generic over number of dimensions

#define MAX_X 30
#define MAX_Y 20
#define MAX_Z 150

template<typename V>
using LatticeMap = array<array<array<V, MAX_Z + 1>, MAX_Y + 1>, MAX_X + 1>;
typedef tuple<size_t, size_t, size_t> LatticePoint;

template<typename V>
using LatticeMapUpdates = map<LatticePoint, V>;

typedef tuple<f32, f32, f32> StatePoint;


#define DEATH_BARRIER (-8191.0f + 2048.0f)


static StatePoint to_state_space(const MarioState &m) {
  return make_tuple(
    (f32) (m.faceAngle[0] + 0x2AAA) / 1000.0f,
    (m.pos[1] - DEATH_BARRIER) / 1000.0f,
    sqrtf(m.forwardVel) * 10.0f
  );
}


static bool lattice_point_in_bounds(const LatticePoint &p) {
  return get<0>(p) < MAX_X && get<1>(p) < MAX_Y && get<2>(p) < MAX_Z;
}

static void assert_lattice_point_in_bounds(const LatticePoint &p) {
  if (!lattice_point_in_bounds(p)) {
    printf("point = (%zu, %zu, %zu)\n", get<0>(p), get<1>(p), get<2>(p));
    assert(false);
  }
}


template<typename V>
static V lmap_get(
  const LatticePoint &p,
  const LatticeMap<V> &lmap,
  const LatticeMapUpdates<V> &updates = LatticeMapUpdates<f32>()
) {
  assert_lattice_point_in_bounds(p);

  auto it = updates.find(p);
  if (it != updates.end()) {
    return (f32) it->second;
  }

  return lmap[get<0>(p)][get<1>(p)][get<2>(p)];
}


template<typename V>
static V lmap_set(
  const LatticePoint &p,
  V v,
  LatticeMap<V> &lmap
) {
  assert_lattice_point_in_bounds(p);

  lmap[get<0>(p)][get<1>(p)][get<2>(p)] = v;
}


template<typename V>
static void lmap_update(LatticeMap<V> &lmap, const LatticeMapUpdates<V> &updates) {
  for (auto &it : updates) {
    lmap_set(it.first, it.second, lmap);
  }
}


static f32 lerp(f32 v0, f32 v1, f32 t) {
  return (1 - t) * v0 + t * v1;
}

template<typename V>
static f32 interpolate(
  const StatePoint &p,
  const LatticeMap<V> &lmap,
  const LatticeMapUpdates<V> &updates = LatticeMapUpdates<f32>()
) {
  LatticePoint p0 = {
    (size_t) (get<0>(p) + 0.5),
    (size_t) (get<1>(p) + 0.5),
    (size_t) (get<2>(p) + 0.5)
  };
  return lmap_get(p0, lmap, updates);

  // size_t x0 = (size_t) get<0>(p);
  // size_t y0 = (size_t) get<1>(p);
  // size_t z0 = (size_t) get<2>(p);

  // size_t x1 = x0 + 1;
  // size_t y1 = y0 + 1;
  // size_t z1 = z0 + 1;

  // f32 tx = get<0>(p) - x0;
  // f32 ty = get<1>(p) - y0;
  // f32 tz = get<2>(p) - z0;

  // f32 v000 = lmap_get({ x0, y0, z0 }, lmap, updates);
  // f32 v001 = lmap_get({ x0, y0, z1 }, lmap, updates);
  // f32 v010 = lmap_get({ x0, y1, z0 }, lmap, updates);
  // f32 v011 = lmap_get({ x0, y1, z1 }, lmap, updates);
  // f32 v100 = lmap_get({ x1, y0, z0 }, lmap, updates);
  // f32 v101 = lmap_get({ x1, y0, z1 }, lmap, updates);
  // f32 v110 = lmap_get({ x1, y1, z0 }, lmap, updates);
  // f32 v111 = lmap_get({ x1, y1, z1 }, lmap, updates);

  // f32 v_00 = lerp(v000, v100, tx);
  // f32 v_01 = lerp(v001, v101, tx);
  // f32 v_10 = lerp(v010, v110, tx);
  // f32 v_11 = lerp(v011, v111, tx);

  // f32 v__0 = lerp(v_00, v_10, ty);
  // f32 v__1 = lerp(v_01, v_11, ty);

  // f32 v___ = lerp(v__0, v__1, tz);

  // return v___;
}


struct RunResult {
  vector<f32> heights;
  vector<StatePoint> path;
};

static RunResult simulate(
  const MarioState &initial_m,
  size_t num_frames,
  const LatticeMap<f32> &stick_y_map,
  const LatticeMapUpdates<f32> &stick_y_updates = LatticeMapUpdates<f32>()
) {
  RunResult result;
  MarioState m = initial_m;

  result.heights.push_back(m.pos[1]);

  for (size_t i = 0; i < num_frames; i++) {
    StatePoint state = to_state_space(m);
    result.path.push_back(state);

    m.controller->stickY = interpolate(state, stick_y_map, stick_y_updates);

    act_flying(&m, true);

    if (m.pos[1] < DEATH_BARRIER) {
      break;
    }
    result.heights.push_back(m.pos[1]);
  }

  return result;
}


static default_random_engine generator;

static f32 random01() {
  return uniform_real_distribution<f32>(0.0, 1.0)(generator);
}


static LatticeMapUpdates<f32> random_map_updates(
  const LatticeMap<f32> &lmap,
  const vector<StatePoint> &focus
) {
  assert(focus.size() > 0);

  LatticeMapUpdates<f32> updates;

  size_t count = 1;
  for (size_t i = 0; i < count; i++) {
    StatePoint root_state = focus[(size_t) (random01() * focus.size())];
    LatticePoint root = {
      (size_t) (get<0>(root_state) + 0.5),
      (size_t) (get<1>(root_state) + 0.5),
      (size_t) (get<2>(root_state) + 0.5)
    };

    f32 change = (2 * random01() - 1) * 32;

    int radius_x = 5;
    int radius_y = 1;
    int radius_z = 5;
    for (int dx = -radius_x; dx <= radius_x; dx++) {
      for (int dy = -radius_y; dy <= radius_y; dy++) {
        for (int dz = -radius_z; dz <= radius_z; dz++) {
          LatticePoint p = make_tuple(
            get<0>(root) + dx,
            get<1>(root) + dy,
            get<2>(root) + dz
          );
          if (!lattice_point_in_bounds(p)) {
            continue;
          }

          f32 udx = (f32) dx / (f32) (radius_x + 1);
          f32 udy = (f32) dy / (f32) (radius_x + 1);
          f32 udz = (f32) dz / (f32) (radius_x + 1);
          f32 u_sq_dist = udx * udx + udy * udy + udz * udz;
          if (u_sq_dist > 1.0f) {
            continue;
          }

          f32 scale = 1.0f - u_sq_dist;
          // f32 scale = 1.0f / (1.0f + (dx*dx + dy*dy + dz*dz) / (radius + 1.0f));

          f32 new_value = lmap_get(p, lmap) + change * scale;
          new_value = min(max(new_value, -64), 64);
          updates[p] = new_value;
        }
      }
    }
  }

  return updates;
}


static vector<f32> get_peaks(const vector<f32> &heights) {
  vector<f32> peaks;
  bool ascending = false;
  f32 extremum = numeric_limits<f32>::infinity();

  for (f32 height : heights) {
    if (ascending) {
      if (height < extremum - 100.0f) {
        peaks.push_back(extremum);
        ascending = false;
        extremum = height;
      } else {
        extremum = max(extremum, height);
      }
    } else {
      if (height > extremum + 100.0f) {
        ascending = true;
        extremum = height;
      } else {
        extremum = min(extremum, height);
      }
    }
  }

  return peaks;
}


typedef tuple<f32, f32, f32> Utility;

static string to_string(Utility utility) {
  return to_string(get<0>(utility)) + ", " +
    to_string(get<1>(utility)) + ", " +
    to_string(get<2>(utility));
}

static Utility get_utility(const vector<f32> &heights) {
  f32 max_height = *max_element(heights.begin(), heights.end());

  vector<f32> peaks = get_peaks(heights);
  f32 average_gain = peaks.size() > 1
    ? (peaks.back() - peaks.front()) / (f32) (peaks.size() - 1)
    : -numeric_limits<f32>::infinity();

  return Utility(max_height, average_gain, heights.size());
}


static void improve_stick_y_map(
  LatticeMap<f32> &stick_y_map,
  const MarioState &initial_m,
  size_t num_frames,
  size_t iterations
) {
  vector<StatePoint> current_path;
  Utility current_utility;

  {
    RunResult result = simulate(initial_m, num_frames, stick_y_map);
    current_path = result.path;
    current_utility = get_utility(result.heights);
  }

  for (size_t i = 0; i < iterations; i++) {
    LatticeMapUpdates<f32> candidate = random_map_updates(stick_y_map, current_path);
    RunResult result = simulate(initial_m, num_frames, stick_y_map, candidate);
    Utility utility = get_utility(result.heights);

    if (utility > current_utility) {
      lmap_update(stick_y_map, candidate);
      current_path = result.path;
      current_utility = utility;
    }
  }
}


static unique_ptr<LatticeMap<f32>> find_continuation(
  const LatticeMap<f32> &initial_stick_y_map,
  const MarioState &initial_m,
  size_t num_frames,
  size_t num_spawns,
  function<void(const vector<f32> &)> callback
) {
  unique_ptr<LatticeMap<f32>> best_stick_y_map;
  Utility best_utility(-numeric_limits<f32>::infinity());

  for (size_t i = 0; i < num_spawns; i++) {
    unique_ptr<LatticeMap<f32>> stick_y_map(new LatticeMap<f32>());
    *stick_y_map = initial_stick_y_map;

    improve_stick_y_map(*stick_y_map, initial_m, num_frames, 1000);

    RunResult result = simulate(initial_m, num_frames, *stick_y_map);
    Utility utility = get_utility(result.heights);

    if (utility > best_utility) {
      best_stick_y_map = move(stick_y_map);
      best_utility = utility;
      printf("%d: Utility = %s\n", i, to_string(best_utility).c_str());
      callback(result.heights);
    } else {
      callback(vector<f32>()); // Enable Ctrl-C
    }
  }

  return best_stick_y_map;
}


void optimize_for_stick_y(
  const MarioState &initial_m,
  size_t num_frames,
  function<void(const vector<f32> &)> callback
) {
  generator.seed(time(nullptr));

  unique_ptr<LatticeMap<f32>> best_stick_y_map(new LatticeMap<f32>());

  printf("[0-1000]\n");
  best_stick_y_map = find_continuation(
    *best_stick_y_map,
    initial_m,
    num_frames,
    100,
    callback
  );

  // printf("[1000-2000]\n");
  // best_stick_y_map = find_continuation(
  //   *best_stick_y_map,
  //   initial_m,
  //   num_frames,
  //   100,
  //   callback
  // );

  // printf("[2000-3000]\n");
  // best_stick_y_map = find_continuation(
  //   *best_stick_y_map,
  //   initial_m,
  //   num_frames,
  //   100,
  //   callback
  // );

  // Utility best_utility(-numeric_limits<f32>::infinity());

  // while (true) {
  //   unique_ptr<LatticeMap<f32>> stick_y_map(new LatticeMap<f32>());
  //   // if (best_stick_y_map && random01() < 0.1f) {
  //   //   *stick_y_map = *best_stick_y_map;
  //   // }

  //   improve_stick_y_map(*stick_y_map, initial_m, num_frames, 1000);

  //   RunResult result = simulate(initial_m, num_frames, *stick_y_map);
  //   Utility utility = get_utility(result.heights);

  //   if (utility > best_utility) {
  //     best_stick_y_map = move(stick_y_map);
  //     best_utility = utility;
  //     printf("Utility = %s\n", to_string(best_utility).c_str());
  //     callback(result.heights);
  //   } else {
  //     callback(vector<f32>()); // Enable Ctrl-C
  //   }

  //   if (get<0>(utility) > 3100) {
  //     break;
  //   }
  // }

  printf("Optimizing\n");

  while (true) {
    RunResult result = simulate(initial_m, num_frames, *best_stick_y_map);
    Utility start_utility = get_utility(result.heights);

    improve_stick_y_map(*best_stick_y_map, initial_m, num_frames, 100);

    result = simulate(initial_m, num_frames, *best_stick_y_map);
    Utility end_utility = get_utility(result.heights);

    if (end_utility > start_utility) {
      printf("Utility = %s\n", to_string(end_utility).c_str());
      callback(result.heights);
    } else {
      callback(vector<f32>()); // Enable Ctrl-C
    }
  }
}
