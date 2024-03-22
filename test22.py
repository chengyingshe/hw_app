import numpy as np
import heapq
import random
import time

# 地图标记
LAND, SEA, OBS, ROB, BERT, TARGET = 0, 1, 2, 3, 4, 5
mark = {
    '.': LAND,  # 空地
    '*': SEA,   # 海洋
    '#': OBS,   # 障碍物
    'A': ROB,   # 机器人
    'B': TARGET # 船舶
}
reverse_mark = {v: k for k, v in mark.items()}

# 初始化地图
f_path = 'map1.txt'
mymap = np.zeros((200, 200), dtype=int)
robots = []

# 读取地图
with open(f_path, 'r') as f:
    for i, line in enumerate(f):
        for j, char in enumerate(line.strip()):
            mymap[i, j] = mark[char]
            if char == 'A':
                robots.append((i, j))

# 定义移动方向
directions = {
    'up': np.array([-1, 0]),
    'down': np.array([1, 0]),
    'left': np.array([0, -1]),
    'right': np.array([0, 1])
}

# 启发式函数 - 曼哈顿距离
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* 算法实现
def a_star(start, goal, map):
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
            if 0 <= neighbor[0] < map.shape[0] and 0 <= neighbor[1] < map.shape[1] and map[neighbor] not in [OBS, ROB]:
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
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]

# 随机初始化货物
def init_goods(mymap, n_goods):
    # 找到所有标记为 LAND 的位置
    free_positions = [(i, j) for i in range(200) for j in range(200) if mymap[i, j] == LAND]
    if len(free_positions) < n_goods:
        raise ValueError("Not enough LAND positions for the number of goods")

    # 从这些位置中随机选择 n_goods 个位置放置货物
    goods_positions = random.sample(free_positions, n_goods)
    for pos in goods_positions:
        mymap[pos] = BERT  # 更新地图，将选中的位置标记为货物（BERT）
    return goods_positions


# 找到所有机器人到所有货物的最短路径
def assign_robots_to_goods(robots, goods, mymap):
    assignments = {}
    # num_robots*num_goods次运算
    for robot in robots:
        shortest_path = None
        closest_good = None
        for good in goods:
            path = a_star(robot, good, mymap)
            if not shortest_path or len(path) < len(shortest_path):
                shortest_path = path
                closest_good = good
        if closest_good:
            assignments[robot] = {'good': closest_good, 'path': shortest_path}
            goods.remove(closest_good)  # 移除已分配的货物
    return assignments

# 找到最近的船舶位置
def find_nearest_ship(position, mymap):
    ship_positions = np.argwhere(mymap == TARGET).tolist()
    ship_positions = [tuple(pos) for pos in ship_positions]
    closest_ship = None
    min_distance = np.inf
    for ship in ship_positions:
        distance = heuristic(position, ship)
        if distance < min_distance:
            min_distance = distance
            closest_ship = ship
    return closest_ship

def simulate(mymap, robots, goods_positions):
    assignments = assign_robots_to_goods(robots, goods_positions, mymap)
    robot_trajectories = {robot: [] for robot in robots}  # 用于存储每个机器人的轨迹

    for robot, info in assignments.items():
        # 记录从机器人到货物的路径
        for step in info['path']:
            robot_trajectories[robot].append(step)  # 添加步骤到轨迹
        
        goods_pos = info['good'] if 'good' in info else None  # 获取货物位置

        # 寻找并记录到最近船舶的路径
        last_position = info['path'][-1] if info['path'] else robot  # 如果已经移动，使用最后一个位置，否则使用初始位置
        ship_pos = find_nearest_ship(last_position, mymap)
        if ship_pos:
            ship_path = a_star(last_position, ship_pos, mymap)
            if ship_path:  # 确保路径存在
                robot_trajectories[robot].extend(ship_path[1:])  # 添加船舶路径到轨迹，排除起点重复

    # 打印每个货物对应的机器人轨迹
    for robot, trajectory in robot_trajectories.items():
        if trajectory:  # 确保轨迹非空
            goods_pos_str = f" to goods at {assignments[robot]['good']}" if robot in assignments and 'good' in assignments[robot] else ""
            print(f"Robot starting from {robot}{goods_pos_str} trajectory:")
            trajectory_str = " -> ".join(map(str, [robot] + trajectory))  # 包含起始点
            print(trajectory_str)
            if 'ship' in assignments[robot]:
                print(f" and then to ship at {assignments[robot]['ship']}")
            print('Finished\n')

n_goods = random.randint(1, 10)  # 随机选择1到10个货物

goods_positions = init_goods(mymap, n_goods)
start = time.time()
simulate(mymap, robots, goods_positions)
end = time.time()
print('time:', end - start, 'n_goods: ', n_goods)