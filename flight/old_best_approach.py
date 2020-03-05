
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

def get_metric_from_heights(heights):
  # first_descending_peak = None
  # max_peak = 0
  # for peak in get_peaks(heights):
  #   if peak < max_peak:
  #     first_descending_peak = peak
  #     break
  #   else:
  #     max_peak = peak
  peaks = get_peaks(heights)
  peak_prefix = []
  for peak in peaks:
    if len(peak_prefix) == 0 or peak > peak_prefix[-1]:
      peak_prefix.append(peak)
    else:
      peak_prefix.append(peak)
      break
  # peak_diff = float('inf') if len(peaks) < 2 else max(peaks) - min(peaks)
  # return -peak_diff, max(heights) #len(heights) #, first_descending_peak or 0 #max(heights), len(heights), first_descending_peak or 0
  return max(heights), peak_prefix, len(heights)

def get_metric(weights):
  heights = C.get_heights_from_weights(weights, CAP_TIME)
  return get_metric_from_heights(heights)

def get_metric_2(controls):
  heights = get_heights(controls)
  return get_metric_from_heights(heights)


initial_controls = random_controls()
best_controls = (initial_controls, get_metric_2(initial_controls))

last_render_time = time.time()

while True:
  candidate_controls = random_mod(best_controls[0])
  candidate_metric = get_metric_2(candidate_controls)

  if candidate_metric > best_controls[1]:
    best_controls = (candidate_controls, candidate_metric)

    print(best_controls[1])
    if time.time() > last_render_time + 0.1:
      graph_heights(get_heights(best_controls[0]))
      last_render_time = time.time()
