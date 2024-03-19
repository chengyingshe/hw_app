import numpy as np
import heapq

# 初始化地图
f_path = 'map1.txt' # 请根据实际路径修改
mymap = np.zeros((200, 200))
LAND, SEA, OBS, ROB, BERT = 0, 1, 2, 3, 4
mark = {
  '.': LAND,
  '*': SEA,
  '#': OBS,
  'A': ROB,
  'B': BERT
}

with open(f_path, 'r') as f:
  for i in range(200): # 获取200x200地图
    line = f.readline().strip()
    chars = list(line)
    for j in range(len(chars)):
      mymap[i, j] = mark[chars[j]]

# 机器人和货物的位置
rob_pos = np.array([108, 74])
gd_pos = np.array([104, 37])
directions = {
  '2': np.array([0, -1]), 
  '1': np.array([-1, 0]), 
  '3': np.array([0, 1]),
  '0': np.array([1, 0])
}

def heuristic(a, b):
  """
  启发式函数 - 曼哈顿距离
  """
  return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, map, directions):
  open_set = []
  heapq.heappush(open_set, (0 + heuristic(tuple(start), tuple(goal)), 0, tuple(start)))
  came_from = {}
  g_score = {tuple(start): 0}
  f_score = {tuple(start): heuristic(tuple(start), tuple(goal))}

  while open_set:
    current = heapq.heappop(open_set)[2]
    if current == tuple(goal):
      return reconstruct_path(came_from, current)

    for direction_value in directions.values():
      neighbor = np.array(current) + direction_value
      if 0 <= neighbor[0] < 200 and 0 <= neighbor[1] < 200 and map[tuple(neighbor)] != OBS:
        tentative_g_score = g_score[current] + 1

        if tuple(neighbor) not in g_score or tentative_g_score < g_score[tuple(neighbor)]:
          came_from[tuple(neighbor)] = current
          g_score[tuple(neighbor)] = tentative_g_score
          f_score[tuple(neighbor)] = g_score[tuple(neighbor)] + heuristic(tuple(neighbor), tuple(goal))
          if tuple(neighbor) not in [i[2] for i in open_set]:
            heapq.heappush(open_set, (f_score[tuple(neighbor)], tentative_g_score, tuple(neighbor)))

  return []

# 路径重建
def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]
