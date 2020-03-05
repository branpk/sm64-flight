import json
import random
import time
import math

import plotly.graph_objects as go
import matplotlib.pyplot as plt
import numpy as np

import c_ext as C

controller = C.Controller()

def new_mario_state():
  m = C.MarioState()
  m.controller = controller
  return m

def copy_mario_state(m):
  m1 = C.MarioState()
  C.copy_mario_state(m1, m)
  return m1


CAP_TIME = 1800
DEATH_BARRIER = -2000


plt.ion()

def graph_heights(heights, ylim=(0, 3500), name=None):
  plt.clf()
  plt.plot(heights)
  plt.xlim(0, CAP_TIME)
  # plt.ylim(0, 3500)
  plt.ylim(*ylim)
  plt.draw()
  plt.pause(0.0001)
  # graphs.append(go.Scatter(name=name, x=list(range(len(heights))), y=heights, mode='lines'))

def graph_heights_multi(heights, name=None):
  plt.clf()
  plt.xlim(0, CAP_TIME)
  plt.ylim(DEATH_BARRIER, 1500)
  for h in heights:
    plt.plot(h)
  plt.draw()
  plt.pause(0.0001)


def get_heights(controls):
  controls = controls[:CAP_TIME]
  m = new_mario_state()
  m.forward_vel = 50
  m.pos.y = 2000
  heights = [m.pos.y]
  for stick_y in controls:
    m.controller.stick_y = stick_y
    C.act_flying(m, True)
    if m.pos.y < 0:
      break
    heights.append(m.pos.y)
  return heights

def get_max_height(controls):
  m = new_mario_state()
  m.forward_vel = 50
  return C.compute_max_height(m, controls, CAP_TIME, DEATH_BARRIER)


def graph(controls, name=None):
  graph_heights(get_heights(controls), name=name)

def graph_multi(controls, name=None):
  graph_heights_multi([get_heights(c) for c in controls], name=name)



def random_controls():
  controls = []
  while len(controls) < CAP_TIME:
    stick_y = (2*random.random() - 1) * 64
    duration = 1 + int(random.random() * 90)
    for _ in range(duration):
      controls.append(stick_y)
  return controls

def random_controls_2():
  controls = []
  while len(controls) < CAP_TIME:
    stick_y = random.choice((-64, 64))
    for _ in range(30):
      controls.append(stick_y)
  return controls

def random_mod(parent):
  controls = list(parent)

  if random.random() < 0.01:
    duration = int(random.random() * 100)
    for i in range(len(controls) - duration, len(controls)):
      controls[i] = -64
    return controls

  num_mods = 1#random.randrange(1, 5)
  for _ in range(num_mods):
    frame = random.randrange(0, len(parent))
    stick_y = (2*random.random() - 1) * 64
    duration = 1 + int(random.random() * 30)
    for i in range(frame, min(frame + duration, len(controls))):
      controls[i] = stick_y
  return controls


# def get_peaks(heights):
#   peaks = []
#   valley = False
#   max_height = None
#   for height in heights:
#     if height < 1500 and not valley:
#       valley = True
#       if max_height is not None:
#         peaks.append(max_height)
#     if height > 2500 and valley:
#       valley = False
#       max_height = height
#     if not valley and max_height is not None and height > max_height:
#       max_height = height
#   return peaks


def get_ascents(heights):
  ascents = []
  ascending = False
  min_height = heights[0]
  max_height = heights[0]
  for height in heights:
    if ascending:
      max_height = max(max_height, height)
      if height < max_height - 200:
        ascents.append(max_height - min_height)
        min_height = height
        ascending = False
    else:
      min_height = min(min_height, height)
      if height > min_height + 200:
        max_height = height
        ascending = True
  return ascents


def get_running_maxes(heights):
  running_maxes = []
  current_max = None
  current_min = heights[0]
  for height in heights:
    if current_max is not None and height < current_max - 100:
      if len(running_maxes) == 0 or current_max > running_maxes[-1]:
        running_maxes.append(current_max)
      current_min = height
      current_max = None
    if current_min is not None and height > current_min + 100:
      current_min = None
      current_max = height
    if current_max is not None:
      current_max = max(height, current_max)
    if current_min is not None:
      current_min = min(height, current_min)
  return running_maxes

def get_peaks(heights):
  running_maxes = []
  current_max = None
  current_min = heights[0]
  for height in heights:
    if current_max is not None and height < current_max - 100:
      # if len(running_maxes) == 0 or current_max > running_maxes[-1]:
      running_maxes.append(current_max)
      current_min = height
      current_max = None
    if current_min is not None and height > current_min + 100:
      current_min = None
      current_max = height
    if current_max is not None:
      current_max = max(height, current_max)
    if current_min is not None:
      current_min = min(height, current_min)
  return running_maxes



def get_peaks_and_valleys(heights):
  peaks = []
  valley = True
  extremum = 0
  for height in heights:
    if (height < -1000 and not valley) or (height > 300 and valley):
      peaks.append(int(-extremum) // 10 if valley else extremum)
      valley = not valley
    if valley:
      extremum = min(extremum, height)
    else:
      extremum = max(extremum, height)
  return peaks


# def get_metric(controls):
#   return get_max_height(controls)
#   # heights = get_heights(controls)
#   # # peaks = get_peaks(heights)
#   # # peaks = get_peaks_and_valleys(heights)
#   # return max(heights) #(max(heights), len(heights))


start_time = time.time()




def get_heights_from_weights(weights):
  m = new_mario_state()
  m.pos.y = 2000
  m.forward_vel = 50

  heights = [m.pos.y]
  for i in range(CAP_TIME):
    features = C.get_features(m, CAP_TIME - i)
    stick_y = min(max(np.dot(weights, features), -64), 64)

    m.controller.stick_y = stick_y
    C.act_flying(m, True)

    if m.pos.y < 0:
      break
    heights.append(m.pos.y)

  return heights

def get_features(m, frames_left):
  # return np.array([
  #   m.face_angle.x,
  #   m.angle_vel.x,
  #   m.pos.y,
  #   m.forward_vel,
  #   1,
  # ], dtype='f4')

  # base_features = [
  #   m.face_angle.x / 0x2AAA,
  #   m.angle_vel.x / 0xA00,
  #   m.pos.y / 1500 - 1,
  #   m.forward_vel / 50 - 1,
  #   frames_left,
  # ]
  base_features = [
    m.face_angle.x,
    m.angle_vel.x,
    m.pos.y,
    m.forward_vel,
    # frames_left,
  ]
  features = list(base_features)
  # for f in base_features:
  #   features.append(math.sqrt(abs(f)))
  #   features.append(math.sin(f))
  #   features.append(math.cos(f))
  #   features.append(abs(f))
  # base_features = list(features)
  # for f1 in base_features:
  #   for f2 in base_features:
  #     features.append(f1 * f2)
  # for f1 in base_features:
  #   for f2 in base_features:
  #     for f3 in base_features:
  #       features.append(f1 * f2 * f3)
  return np.array(features + [1])

# def get_metric_from_heights(heights):
#   # first_descending_peak = None
#   # max_peak = 0
#   # for peak in get_peaks(heights):
#   #   if peak < max_peak:
#   #     first_descending_peak = peak
#   #     break
#   #   else:
#   #     max_peak = peak
#   peaks = get_peaks(heights)
#   peak_prefix = []
#   for peak in peaks:
#     if len(peak_prefix) == 0 or peak > peak_prefix[-1]:
#       peak_prefix.append(peak)
#     else:
#       peak_prefix.append(peak)
#       break
#   # peak_diff = float('inf') if len(peaks) < 2 else max(peaks) - min(peaks)
#   # return -peak_diff, max(heights) #len(heights) #, first_descending_peak or 0 #max(heights), len(heights), first_descending_peak or 0
#   return max(heights), peak_prefix, len(heights)

# def get_metric(weights):
#   heights = C.get_heights_from_weights(weights, CAP_TIME)
#   return get_metric_from_heights(heights)

# def get_metric_2(controls):
#   heights = get_heights(controls)
#   return get_metric_from_heights(heights)


# initial_controls = random_controls()
# best_controls = (initial_controls, get_metric_2(initial_controls))

# last_render_time = time.time()

# while True:
#   candidate_controls = random_mod(best_controls[0])
#   candidate_metric = get_metric_2(candidate_controls)

#   if candidate_metric > best_controls[1]:
#     best_controls = (candidate_controls, candidate_metric)

#     print(best_controls[1])
#     if time.time() > last_render_time + 0.1:
#       graph_heights(get_heights(best_controls[0]))
#       last_render_time = time.time()




ranges = [[float('inf'), float('-inf')]] * 3
def get_info(m, frame):
  info = (
    m.face_angle.x, # ~ [-10922, 10410]
    # m.angle_vel.x, # ~ [-269, 550]
    # m.pos.y,
    m.forward_vel, # ~ [15, 174]
    # frame,
  )
  # for i in range(3):
  #   start_range = list(ranges[i])
  #   ranges[i] = [min(ranges[i][0], info[i]), max(ranges[i][1], info[i])]
  #   if ranges[i] != start_range:
  #     print(f'{0} -> {ranges[0]}')
  #     print(f'{1} -> {ranges[1]}')
  #     print(f'{2} -> {ranges[2]}')
  return info

# TODO: Interpolate instead of using buckets
def get_bucket(info):
  return (
    info[0] // 100 // 10,
    # info[1] // 100, #info[1] // 7 // 10,
    # int(info[1]) // 100 // 10,
    int(math.sqrt(info[1]) * 10),
    # info[2] // 40,
  )

# 124541
# 0 -> [-10922, 10410]
# 1 -> [-378, 642]
# 2 -> [26.865854263305664, 176.5796356201172]

def run_strategy(speed, frames, strategy, strategy_updates={}):
  m = new_mario_state()
  m.forward_vel = speed
  m.pos.y = 0

  heights = [m.pos.y]
  relevant_buckets = []

  for i in range(frames):
    bucket = get_bucket(get_info(m, i))
    relevant_buckets.append(bucket)

    stick_y = strategy_updates.get(bucket)
    if stick_y is None:
      stick_y = strategy.get(bucket, 0)

    m.controller.stick_y = stick_y
    C.act_flying(m, True)

    # if m.pos.y < -8191 + 2048:
    #   break
    heights.append(m.pos.y)

  return heights, relevant_buckets

def random_strategy_update(strategy, relevant_buckets):
  updates = {}
  for _ in range(1):
    root_bucket = random.choice(relevant_buckets)
    change = (2*random.random() - 1) * 32
    R = 5
    for dx in range(-R, R + 1):
      for dy in range(-R, R + 1):
        # for dz in range(-R, R + 1):
          bucket = (root_bucket[0] + dx, root_bucket[1] + dy)#, root_bucket[2] + dz)
          scale = 1/(1 + (dx*dx + dy*dy + 0) / (R + 1))
          updates[bucket] = strategy.get(bucket, 0) + change * scale
          updates[bucket] = min(max(updates[bucket], -64), 64)
    # {
    #   # bucket: min(max(strategy.get(bucket, 0) + (2*random.random() - 1) * 16, -64), 64)
    #   bucket: -64,
    # }
  return updates

def get_metric(heights):
  peaks = get_peaks(heights)
  avg_gain = (peaks[-1] - peaks[0]) / (len(peaks) - 1) if len(peaks) > 1 else float('-inf')
  # max_gain = max((peaks[i + 1] - peaks[i] for i in range(len(peaks) - 1)), default=float('-inf'))
  # min_gain = min((peaks[i + 1] - peaks[i] for i in range(len(peaks) - 1)), default=float('-inf'))
  return max(heights), avg_gain, len(heights)    # max(heights), peaks, len(heights)


# m = new_mario_state()
# m.forward_vel = 113
# m.pos.y = -2200

# ylim = [-1000, 1000]
# last_render_time = time.time() - 1
# def callback(heights):
#   global last_render_time
#   if len(heights) == 0:
#     return
#   ylim[0] = min(max(ylim[0], -5000), min(heights))
#   ylim[1] = max(ylim[1], max(heights))
#   if time.time() > last_render_time + 0.1:
#     graph_heights(heights, ylim=ylim)
#     last_render_time = time.time()

# C.optimize_for_stick_y(m, 1800, callback)
# quit()




def get_peaks(heights):
  running_maxes = []
  current_max = None
  current_min = heights[0]
  for height in heights:
    if current_max is not None and height < current_max - 100:
      # if len(running_maxes) == 0 or current_max > running_maxes[-1]:
      running_maxes.append(current_max)
      current_min = height
      current_max = None
    if current_min is not None and height > current_min + 100:
      current_min = None
      current_max = height
    if current_max is not None:
      current_max = max(height, current_max)
    if current_min is not None:
      current_min = min(height, current_min)
  return running_maxes


ranges = [[float('inf'), float('-inf')]] * 3
def get_info(m):
  info = (
    m.face_angle.x, # ~ [-10922, 10410]
    # m.angle_vel.x, # ~ [-269, 550]
    m.pos.y,
    m.forward_vel, # ~ [15, 174]
  )
  # for i in range(3):
  #   start_range = list(ranges[i])
  #   ranges[i] = [min(ranges[i][0], info[i]), max(ranges[i][1], info[i])]
  #   if ranges[i] != start_range:
  #     print(f'{0} -> {ranges[0]}')
  #     print(f'{1} -> {ranges[1]}')
  #     print(f'{2} -> {ranges[2]}')
  return info

# TODO: Interpolate instead of using buckets
def get_bucket(info):
  return (
    (info[0] + 0x2AAA) // 1000,
    # info[1] // 100, #info[1] // 7 // 10,
    int(info[1] - (-8191 + 2048)) // 1000,
    int(math.sqrt(info[2]) * 10),
  )

# 124541
# 0 -> [-10922, 10410]
# 1 -> [-378, 642]
# 2 -> [26.865854263305664, 176.5796356201172]

def run_strategy(frames, strategy, strategy_updates={}):
  m = new_mario_state()
  m.forward_vel = 113
  m.pos.y = -2200

  heights = [m.pos.y]
  relevant_buckets = []

  for _ in range(frames):
    bucket = get_bucket(get_info(m))
    relevant_buckets.append(bucket)

    stick_y = strategy_updates.get(bucket)
    if stick_y is None:
      stick_y = strategy.get(bucket, 0)

    m.controller.stick_y = stick_y
    C.act_flying(m, True)

    if m.pos.y < -8191 + 2048:
      break
    heights.append(m.pos.y)

  return heights, relevant_buckets

def random_strategy_update(strategy, relevant_buckets):
  updates = {}
  for _ in range(1):
    root_bucket = random.choice(relevant_buckets)
    change = (2*random.random() - 1) * 32
    R = 5
    for dx in range(-R, R + 1):
      for dy in range(-R, R + 1):
        for dz in range(-R, R + 1):
          bucket = (root_bucket[0] + dx, root_bucket[1] + dy, root_bucket[2] + dz)
          scale = 1/(1 + (dx*dx + dy*dy + dz*dz) / (R + 1))
          updates[bucket] = strategy.get(bucket, 0) + change * scale
          updates[bucket] = min(max(updates[bucket], -64), 64)
    # {
    #   # bucket: min(max(strategy.get(bucket, 0) + (2*random.random() - 1) * 16, -64), 64)
    #   bucket: -64,
    # }
  return updates

def get_metric(heights):
  peaks = get_peaks(heights)
  avg_gain = (peaks[-1] - peaks[0]) / (len(peaks) - 1) if len(peaks) > 1 else float('-inf')
  # max_gain = max((peaks[i + 1] - peaks[i] for i in range(len(peaks) - 1)), default=float('-inf'))
  # min_gain = min((peaks[i + 1] - peaks[i] for i in range(len(peaks) - 1)), default=float('-inf'))
  return max(heights), avg_gain, len(heights)    # max(heights), peaks, len(heights)


# CAP_TIME = 1800

# initial_strategy = {}
# initial_heights, initial_relevant_buckets = run_strategy(CAP_TIME, initial_strategy)
# best = (initial_strategy, initial_relevant_buckets, get_metric(initial_heights))

# ylim = [-1000, 1000]
# ylim[0] = min(ylim[0], min(initial_heights))
# ylim[1] = max(ylim[1], max(initial_heights))
# graph_heights(initial_heights, ylim=ylim)
# # print(initial_heights, sum(initial_heights), get_metric(initial_heights))

# last_render_time = time.time()

# while True:
#   candidate_update = random_strategy_update(best[0], best[1])
#   candidate_heights, candidate_relevant_buckets = run_strategy(CAP_TIME, best[0], candidate_update)
#   candidate_metric = get_metric(candidate_heights)

#   # print(candidate_heights, sum(candidate_heights), candidate_metric > best[2])
#   # break

#   if candidate_metric > best[2]:
#     best[0].update(candidate_update)
#     best = (best[0], candidate_relevant_buckets, candidate_metric)

#     print(len(best[0]), candidate_metric)
#     if time.time() > last_render_time + 0.1:
#       ylim[0] = min(max(ylim[0], -5000), min(candidate_heights))
#       ylim[1] = max(ylim[1], max(candidate_heights))
#       graph_heights(candidate_heights, ylim=ylim)
#       last_render_time = time.time()







# CAP_TIME = 1800

# if True:
#   initial_strategy = {}

#   # with open('strategy.json', 'r') as f:
#   #   old_strategy = json.load(f)
#   # for bucket, stick_y in old_strategy:
#   #   new_bucket = (
#   #     bucket[0] * 10,
#   #     bucket[1] * 10,
#   #   )
#   #   for i in range(new_bucket[0], new_bucket[0] + 10):
#   #     for j in range(new_bucket[1], new_bucket[1] + 10):
#   #       initial_strategy[(i, j)] = stick_y

#   starting_speed = 30 + random.random() * 100

#   initial_heights, initial_relevant_buckets = run_strategy(starting_speed, CAP_TIME, initial_strategy)
#   best = (initial_strategy, initial_relevant_buckets, get_metric(initial_heights))

#   ylim = [-1000, 1000]
#   ylim[0] = min(ylim[0], min(initial_heights))
#   ylim[1] = max(ylim[1], max(initial_heights))
#   graph_heights(initial_heights, ylim=ylim)
#   # print(initial_heights, sum(initial_heights), get_metric(initial_heights))

#   last_render_time = time.time()
#   last_switch_time = time.time()

#   while True:
#     candidate_update = random_strategy_update(best[0], best[1])
#     candidate_heights, candidate_relevant_buckets = run_strategy(starting_speed, CAP_TIME, best[0], candidate_update)
#     candidate_metric = get_metric(candidate_heights)

#     # print(candidate_heights, sum(candidate_heights), candidate_metric > best[2])
#     # break

#     if candidate_metric > best[2]:
#       best[0].update(candidate_update)

#       if time.time() > last_switch_time + 10:
#         starting_speed = 30 + random.random() * 100
#         candidate_heights, candidate_relevant_buckets = run_strategy(starting_speed, CAP_TIME, best[0])
#         candidate_metric = get_metric(candidate_heights)
#         print(f'Switch: {starting_speed} -> {candidate_metric}')
#         last_switch_time = time.time()

#       best = (best[0], candidate_relevant_buckets, candidate_metric)

#       print(len(best[0]), candidate_metric)
#       if time.time() > last_render_time + 0.1:
#         ylim[0] = min(max(ylim[0], -5000), min(candidate_heights))
#         ylim[1] = max(ylim[1], max(candidate_heights))
#         graph_heights(candidate_heights, ylim=ylim)
#         last_render_time = time.time()


# with open('strategy.json', 'r') as f:
#   strategy = {tuple(k): v for k, v in json.load(f)}

# def to_info(bucket):
#   pitch = bucket[0] * 1000
#   speed = (bucket[1] / 10) ** 2
#   return (pitch, speed)

# pitch_range = (
#   min(to_info(bucket)[0] for bucket in strategy),
#   max(to_info(bucket)[0] for bucket in strategy) + 1,
# )
# speed_range = (
#   min(to_info(bucket)[1] for bucket in strategy),
#   max(to_info(bucket)[1] for bucket in strategy) + 1,
# )

# graphs = []

# def graph_fixed_pitch(pitch):
#   points = []
#   for bucket, stick_y in sorted(strategy.items()):
#     info = to_info(bucket)
#     if info[0] == pitch:
#       points.append((info[1], stick_y))
#   points.sort()
#   x, y = zip(*points)
#   graphs.append(go.Scatter(name='p=0x%X' % pitch, x=x, y=y, mode='lines'))

# def graph_fixed_speed(speed):
#   points = []
#   for bucket, stick_y in sorted(strategy.items()):
#     info = to_info(bucket)
#     if info[1] == speed:
#       points.append((info[0], stick_y))
#   points.sort()
#   x, y = zip(*points)
#   graphs.append(go.Scatter(name=f'v={speed}', x=x, y=y, mode='lines'))

# # for pitch in range(*pitch_range, 7000):
# #   graph_fixed_pitch(pitch)

# speed_index_range = (
#   min(bucket[1] for bucket in strategy),
#   max(bucket[1] for bucket in strategy) + 1,
# )
# for speed_index in range(*speed_index_range, 5):
#   speed = (speed_index / 10) ** 2
#   graph_fixed_speed(speed)


# print(len(strategy))


# quit()


# num_features = len(get_features(new_mario_state(), 0))
# # initial_weights = np.random.randn(num_features)
# initial_weights = np.array([0] * num_features, dtype='f4')
# best_weights = (initial_weights, get_metric(initial_weights))
# graph_heights(C.get_heights_from_weights(initial_weights, CAP_TIME))


# # for i in range(1000):
# #   candidate_weights = np.random.randn(num_features)
# #   candidate_metric = get_metric(candidate_weights)
# #   if candidate_metric > best_weights[1]:
# #     best_weights = (candidate_weights, candidate_metric)

# #     print(best_weights[0])
# #     print(best_weights[1])
# #     graph_heights(get_heights_from_weights(best_weights[0]))


# while True:
#   for i in range(num_features):
#     basis = np.zeros(num_features, dtype='f4')
#     basis[i] = 1

#     step = basis * np.random.randn() * 0.5 * (abs(best_weights[0][i]) + 0.01)

#     candidate_weights = best_weights[0] + step
#     candidate_metric = get_metric(candidate_weights)
#     if candidate_metric > best_weights[1]:
#       best_weights = (candidate_weights, candidate_metric)

#       print(best_weights[0])
#       print(best_weights[1])
#       graph_heights(get_heights_from_weights(best_weights[0]))




# best = ([], get_metric([]))

# for _ in range(10000):
#   controls = random_controls()
#   metric = get_metric(controls)
#   if metric > best[1]:
#     print('max =', metric)
#     best = (controls, metric)
#     graph(best[0])

# print('Random modifications')

# while True:
#   controls = random_mod(best[0])
#   metric = get_metric(controls)
#   if metric > best[1]:# or random.random() < 0.01:
#     best = (controls, metric)
#     graph(best[0])
#     print('Best =', best[1])
# graph(best[0])



# alive = []
# last_render = time.time()

# while True:
#   old_max = max((a[1] for a in alive), default=0)

#   changed = False
#   if len(alive) > 0 and random.random() < 0.9999:
#     index = random.randrange(len(alive))
#     controls = random_mod(alive[index][0])
#     metric = get_metric(controls)
#     if metric > alive[index][1]:
#       alive[index] = (controls, metric)
#     else:
#       continue
#   else:
#     if len(alive) == 0:
#       controls = random_controls()
#     else:
#       index = max(range(len(alive)), key=lambda i: alive[i][1])
#       controls = random_mod(alive[index][0])
#     # controls = random_controls()
#     metric = get_metric(controls)
#     if len(alive) < 10:
#       alive.append((controls, metric))
#     else:
#       index = min(range(len(alive)), key=lambda i: alive[i][1])
#       alive[index] = (controls, metric)

#   new_max = max(a[1] for a in alive)
#   if old_max != new_max:
#     print(new_max)

#   if time.time() - last_render > 0.1:
#     graph_multi([a[0] for a in alive])
#     last_render = time.time()



# alive = []
# last_render = time.time()

# while len(alive) < 3:
#   controls = random_controls()
#   alive.append((controls, get_metric(controls)))

# while True:
#   old_max = max((a[1] for a in alive), default=0)

#   index = random.randrange(len(alive))
#   controls = random_mod(alive[index][0])
#   metric = get_metric(controls)
#   if metric > alive[index][1]:
#     alive[index] = (controls, metric)
#   else:
#     continue

#   avg = [sum(alive[i][0][k] for i in range(len(alive))) / len(alive) for k in range(CAP_TIME)]

#   new_max = max(a[1] for a in alive)
#   if old_max != new_max:
#     print('max =', new_max)
#     print('avg =', get_metric(avg))

#   if time.time() - last_render > 0.1:
#     graph_multi([a[0] for a in alive] + [avg])
#     last_render = time.time()


# granularity = 25

# best = ([], get_metric([]))
# max_state = 1 << (CAP_TIME // granularity + 1)

# m = new_mario_state()

# for state in range(max_state):
#   bits = [(state & (1 << i)) != 0 for i in range(CAP_TIME // granularity + 1)]
#   if state % 10000 == 0:
#     print(f'{state}/{max_state}')

#   controls = []
#   for b in bits:
#     control = 64 if b else -64
#     controls += [control] * granularity
#   assert len(controls) >= CAP_TIME

#   m.forward_vel = 50
#   metric = C.compute_max_height(m, controls, CAP_TIME, DEATH_BARRIER)
#   if metric > best[1]:
#     print(metric)
#     best = (controls, metric)
#     graph(best[0])


def approx_max_height(m, frames, granularity):
  max_height = m.pos.y
  max_state = 1 << (frames // granularity + 1)

  for state in range(max_state):
    # if state % 100000 == 0:
    #   print(f'{state} / {max_state}')
    bits = [(state & (1 << i)) != 0 for i in range(frames // granularity + 1)]

    controls = []
    for b in bits:
      control = 64 if b else -64
      controls += [control] * granularity
    assert len(controls) >= frames

    m1 = copy_mario_state(m)
    height = C.compute_max_height(m1, controls, frames, DEATH_BARRIER)
    if height > max_height:
      max_height = height

  return max_height

def best_control(m, heuristic):
  options = [64, 32, 0, -32, -64]
  best_option = None

  for control in options:
    m1 = copy_mario_state(m)
    m1.controller.stick_y = control
    C.act_flying(m1, True)

    metric = heuristic(m1)
    if best_option is None or metric > best_option[2]:
      best_option = (control, m1, metric)

  return (best_option[0], best_option[1])


def controls_from_heuristic(m, heuristic):
  controls = []
  for i in range(CAP_TIME):
    # if i % 10 == 0:
    print(f'{i}/{CAP_TIME}')
    control, m = best_control(m, lambda m1: heuristic(m1, i))
    controls.append(control)

  return controls
  # print(get_max_height(controls))
  # graph(controls)


# m = new_mario_state()
# m.forward_vel = 50
# # print(approx_max_height(m, 19, 1))

# # print(C.approx_max_height(m, 20, DEATH_BARRIER))


# def heuristic(m):
#   m1 = copy_mario_state(m)
#   return C.approx_max_height(m1, 100, 10, DEATH_BARRIER)


# m = new_mario_state()
# m.forward_vel = 50

# controls = []
# for i in range(CAP_TIME):
#   # if i % 10 == 0:
#   print(f'{i}/{CAP_TIME}')
#   control, m = best_control(m, heuristic)
#   controls.append(control)

# print(get_max_height(controls))
# graph(controls)


# heuristic_cache = {}

# time_granularity = 18
# collisions = [0, 0]

# def get_bucket(info):
#   return (
#     info[0],
#     info[1],
#     info[2],
#     info[3],
#     info[4],
#   )

# def heuristic(m, frame):
#   info = (
#     m.face_angle.y,
#     m.angle_vel.y,
#     m.pos.y - DEATH_BARRIER,
#     m.forward_vel,
#     CAP_TIME - frame,
#   )
#   bucket = get_bucket(info)
#   # print(bucket)
#   h = heuristic_cache.get(bucket)
#   collisions[1] += 1
#   if h is not None:
#     collisions[0] += 1
#     return h

#   h = m.pos.y - DEATH_BARRIER
#   if frame >= CAP_TIME or h < 0:
#     return h

#   options = [64, -64]

#   for control in options:
#     m1 = copy_mario_state(m)
#     m1.controller.stick_y = control
#     for _ in range(time_granularity):
#       C.act_flying(m1, True)
#       h = max(h, m1.pos.y - DEATH_BARRIER)

#     h = max(h, heuristic(m1, frame + time_granularity))

#   heuristic_cache[bucket] = h
#   return h


# for forward_vel in [10 + 10 * x for x in range(5)]:
#   for pos_y in [-500 + 100 * x for x in range(10)]:
#     m = new_mario_state()
#     m.pos.y = pos_y
#     m.forward_vel = forward_vel

#     print(forward_vel, pos_y)
#     print('h =', heuristic(m, 0) + DEATH_BARRIER)
#     print(f'{collisions[0]}/{collisions[1]} = {collisions[0] / collisions[1]}')


# with open('h_accurate.json', 'w') as f:
#   json.dump(list(heuristic_cache.items()), f)

# with open('h_accurate.json', 'r') as f:
#   h_accurate = json.load(f)
# def heu(info):
#   m = new_mario_state()
#   m.face_angle.y = info[0]
#   m.angle_vel.y = info[1]
#   m.pos.y = DEATH_BARRIER + info[2]
#   m.forward_vel = info[3]
#   frame = CAP_TIME - info[4]
#   return heuristic(m, frame)
# errors = [abs(heu(info) - h) for info, h in h_accurate if random.randrange(100) == 0]
# print('max error =', max(errors))

# print('h =', heuristic(m, 0) + DEATH_BARRIER)
# print(f'{collisions[0]}/{collisions[1]} = {collisions[0] / collisions[1]}')
# controls = controls_from_heuristic(m, heuristic)
# print('Y =', get_max_height(controls))
# graph(controls)





print('Elapsed:', time.time() - start_time)
# go.Figure(data=graphs).show()
plt.show(block=True)
