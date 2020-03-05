
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
    info[0] // 100 // 10,
    # info[1] // 100, #info[1] // 7 // 10,
    int(info[1]) // 100 // 10,
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


CAP_TIME = 1800

initial_strategy = {}
initial_heights, initial_relevant_buckets = run_strategy(CAP_TIME, initial_strategy)
best = (initial_strategy, initial_relevant_buckets, get_metric(initial_heights))

ylim = [-1000, 1000]
ylim[0] = min(ylim[0], min(initial_heights))
ylim[1] = max(ylim[1], max(initial_heights))
graph_heights(initial_heights, ylim=ylim)
# print(initial_heights, sum(initial_heights), get_metric(initial_heights))

last_render_time = time.time()

while best[2][0] < 3000:
  candidate_update = random_strategy_update(best[0], best[1])
  candidate_heights, candidate_relevant_buckets = run_strategy(CAP_TIME, best[0], candidate_update)
  candidate_metric = get_metric(candidate_heights)

  # print(candidate_heights, sum(candidate_heights), candidate_metric > best[2])
  # break

  if candidate_metric > best[2]:
    best[0].update(candidate_update)
    best = (best[0], candidate_relevant_buckets, candidate_metric)

    print(len(best[0]), candidate_metric)
    if time.time() > last_render_time + 0.1:
      ylim[0] = min(max(ylim[0], -5000), min(candidate_heights))
      ylim[1] = max(ylim[1], max(candidate_heights))
      graph_heights(candidate_heights, ylim=ylim)
      last_render_time = time.time()
