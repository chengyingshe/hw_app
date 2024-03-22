import re
import time
import numpy as np
import heapq

# 初始化地图
f_path = 'map1.txt' # 请根据实际路径修改
mymap = np.zeros((200, 200))
LAND, SEA, OBS, ROB, BERTH, GOOD = 0, 1, 2, 3, 4, 5
mark = {
  '.': LAND,
  '*': SEA,
  '#': OBS,
  'A': ROB,
  'B': BERTH
}

with open(f_path, 'r') as f:
  for i in range(200): # 获取200x200地图
    line = f.readline().strip()
    chars = list(line)
    for j in range(len(chars)):
      mymap[i, j] = mark[chars[j]]


directions = {
  '2': (0, -1), 
  '1': (-1, 0), 
  '3': (0, 1),
  '0': (1, 0)
}
reverse_directions = {}
for k, v in directions.items():
    reverse_directions[v] = k

def decode_order(n_order: str):
    """
    向指令字符串中解析新的指令并添加其中
    `n_order`: r0 move[0-3]|'', r0 get, r0 pull, s0 g(0)
    """
    assert re.match(r'[rs]\d\s\w+', n_order), 'Wrong order!'
    l = n_order.split(' ')
    id = int(n_order[1])
    if n_order[0] == 'r':
        if l[1] in ['get', 'pull']:
            return f'{l[1]} {id}\n'
        elif re.match(r'move[0-3]*', l[1]):
            if len(l[1]) == 4: return ''
            return f'move {id} {l[1][4]}\n'
    elif n_order[0] == 's':
        if l[1] == 'g':
            return f'go {id}\n'  # 船去虚拟点
        elif re.match(r'g\d', l[1]):
            return f'ship {id} {int(l[1][1])}\n'
    raise ValueError('Wrong order!')

def get_orders_from_path(path, rob_id):
    orders = []
    for i in range(len(path) - 1):
        direct = reverse_directions[tuple(np.array(path[i + 1]) - np.array(path[i]))]
        orders.append(f'move {rob_id} {direct}')
    return orders

def heuristic(a, b):
  """
  启发式函数 - 曼哈顿距离
  """
  return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* 算法实现
def a_star(start, goal, map, routes: list = [LAND, GOOD, BERTH], blocks: list = []):
    width, height = map.shape[1], map.shape[0]
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[2]
        if current == goal:
            return reconstruct_path(came_from, current)
        for dir in directions.values():
            neighbor = tuple(np.array(current) + dir)
            if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height and \
                    map[neighbor[1], neighbor[0]] in routes and neighbor not in blocks:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    if neighbor not in [i[2] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], tentative_g_score, neighbor))
    return []


# 路径重建
def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:  # 从终点反向寻找起点
        current = came_from[current]
        total_path.append(current)
  

    return total_path[::-1]

# 机器人和货物的位置
rob_pos = (75, 64)
# gd_pos = (108, 74)
gd_pos = (87, 41)
s = time.time()
path = a_star(rob_pos, gd_pos, mymap, blocks=[(75, 63), (75, 62)])
orders = get_orders_from_path(path, 0)
print(f'path: {path}')
print(f'orders: {orders}')
print('time:', time.time() - s)