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
DEATH_BARRIER = -8191 + 2048


plt.ion()

def draw_graph(heights):
  plt.clf()
  plt.xlim(0, CAP_TIME)
  plt.ylim(DEATH_BARRIER, 5000)
  if len(heights) > 0 and (isinstance(heights[0], float) or isinstance(heights[0], int)):
    heights = [heights]
  for h in heights:
    plt.plot(h)
  plt.draw()
  plt.pause(0.0001)

start_time = time.time()


def get_features(m):
  return (
    m.face_angle.x / 1000,
    m.pos.y / 1000,
    math.sqrt(m.forward_vel) * 10,
  )

def feature_sq_dist(f1, f2):
  return sum((f1[i] - f2[i])**2 for i in range(len(f1)))


# TODO: What is this estimating? Max height over N frames after given state? Maybe include frameNum in features?

class ValueMap:
  def __init__(self):
    self.entries = []

  # def estimate(self, m):
  #   if len(self.entries) == 0:
  #     return (0, float('inf'))

  #   features = get_features(m)
  #   nearest = min(self.entries, key=lambda e: feature_sq_dist(features, e[0]))

  #   return nearest[1], math.sqrt(feature_sq_dist(features, nearest[0]))

  # def set_value(self, m, value):
  #   self.entries.append((get_features(m), value))

  def estimate(self, m):
    if len(self.entries) == 0:
      return (0, float('inf'))

    features = get_features(m)
    nearest = min(self.entries, key=lambda e: feature_sq_dist(features, e[1]))

    return nearest[2], math.sqrt(feature_sq_dist(features, nearest[1]))

  def set_value(self, m, value):
    self.entries.append((copy_mario_state(m), get_features(m), value))


# TODO: Could improve performance by storing action information in ValueMap
def find_best_action(vmap, m):
  def action_value(stick_y):
    m1 = copy_mario_state(m)
    m1.controller.stick_y = stick_y
    C.act_flying(m1, True)
    if m1.pos.y < DEATH_BARRIER:
      return -float('inf')
    return vmap.estimate(m1)[0] # TODO: Use distance in some way?

  actions = reversed([-64, -48, -32, -16, 0, 16, 32, 48, 64])
  action_values = map(lambda a: (a, action_value(a)), actions)
  return max(action_values, key=lambda e: e[1])


def update_value_map(vmap, initial_m, num_frames):
  m = copy_mario_state(initial_m)
  for i in range(num_frames):
    stick_y, value1 = find_best_action(vmap, m)
    value = max(m.pos.y, value1)
    vmap.set_value(m, value)

    m.controller.stick_y = stick_y
    C.act_flying(m, True)

    if m.pos.y < DEATH_BARRIER:
      break


def fill_vmap_for_path(vmap, initial_m, num_frames, accept_radius):
  m = copy_mario_state(initial_m)
  for i in range(num_frames):
    est_value, est_dist = vmap.estimate(m)
    stick_y, value1 = find_best_action(vmap, m)

    if est_dist > accept_radius:
      value = max(m.pos.y, value1)
      vmap.set_value(m, value)

    m.controller.stick_y = stick_y
    C.act_flying(m, True)

    if m.pos.y < DEATH_BARRIER:
      break


def run_policy(vmap, initial_m, num_frames):
  m = copy_mario_state(initial_m)
  heights = [m.pos.y]
  for i in range(num_frames):
    stick_y, _ = find_best_action(vmap, m)

    m.controller.stick_y = stick_y
    C.act_flying(m, True)

    if m.pos.y < DEATH_BARRIER:
      break
    heights.append(m.pos.y)
  return heights


def improve_vmap(vmap):
  k = 0
  while True:
    k += 1
    # print(f'Iteration {k}')
    any_improved = False

    for i, (m, features, old_value) in enumerate(vmap.entries):
      _, value1 = find_best_action(vmap, m)
      new_value = max(m.pos.y, value1)

      if new_value > old_value:
        any_improved = True
        vmap.entries[i] = (m, features, new_value)

    if not any_improved:
      break


m = new_mario_state()
m.forward_vel = 113
m.pos.y = -2200
num_frames = 100

vmap = ValueMap()
fill_vmap_for_path(vmap, m, num_frames, 1)
print(max(run_policy(vmap, m, num_frames)))
improve_vmap(vmap)
print(max(run_policy(vmap, m, num_frames)))
fill_vmap_for_path(vmap, m, num_frames, 1)
print(len(vmap.entries))
quit()

heights = [m.pos.y]
for i in range(num_frames):
  stick_y, _ = find_best_action(vmap, m)

  m.controller.stick_y = stick_y
  C.act_flying(m, True)

  if m.pos.y < DEATH_BARRIER:
    break
  heights.append(m.pos.y)

draw_graph(heights)


print('Elapsed:', time.time() - start_time)
# go.Figure(data=graphs).show()
plt.show(block=True)
