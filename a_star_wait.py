import sys
import numpy as np
import heapq
import random

LAND, SEA, OBS, ROB, BERT, GOOD = 0, 1, 2, 3, 4, 5
mark = {  # 地图中的字符含义
    '.': LAND,
    '*': SEA,
    '#': OBS,
    'A': ROB,
    'B': BERT,
    '$': GOOD
}

reverse_mark = {}
for k, v in mark.items():
    reverse_mark[v] = k


def Init_test():
    # 初始化地图
    f_path = 'map1.txt'
    mymap = np.zeros((200, 200), dtype=int)
    robots = []

    # 读取地图
    with open(f_path, 'r') as f:
        for i in range(200): # 获取200x200地图
            line = f.readline().strip()
            chars = list(line)
            for j in range(200):
                mymap[i, j] = mark[chars[j]]
                if chars[j] == reverse_mark[ROB]:
                    robots.append((i, j))

    return mymap, robots

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
    ship_positions = np.argwhere(mymap == BERT).tolist()
    ship_positions = [tuple(pos) for pos in ship_positions]
    closest_ship = None
    min_distance = np.inf
    for ship in ship_positions:
        distance = heuristic(position, ship)
        if distance < min_distance:
            min_distance = distance
            closest_ship = ship
    return closest_ship

def move_instruction(prev, next):
    if next[0] == prev[0] + 1:
        return '3'  # 下移
    elif next[0] == prev[0] - 1:
        return '2'  # 上移
    elif next[1] == prev[1] + 1:
        return '0'  # 右移
    elif next[1] == prev[1] - 1:
        return '1'  # 左移
    return ''

def simulate(mymap, robots, goods_positions):
    assignments = assign_robots_to_goods(robots, goods_positions, mymap)
    robot_moves_map = {}  # 用于存储每个货物所需的机器人路径
    initial_positions = []  # 用于存储每个货物初始位置和对应的机器人初始位置

    for robot, info in assignments.items():
        moves = []  # 存储当前货物所需的机器人路径

        # 从机器人到货物的路径转换为移动指令
        for i in range(1, len(info['path'])):
            instruction = move_instruction(info['path'][i-1], info['path'][i])
            moves.append(instruction)

        moves.append('get')  # 到达货物位置

        # 寻找并记录到最近船舶的移动指令
        if 'path' in info and info['path']:
            last_position = info['path'][-1]  # 使用最后一个位置
            ship_pos = find_nearest_ship(last_position, mymap)
            if ship_pos:
                ship_path = a_star(last_position, ship_pos, mymap)
                for i in range(1, len(ship_path)):
                    instruction = move_instruction(ship_path[i-1], ship_path[i])
                    moves.append(instruction)
                moves.append('pull')  # 到达船舶位置

        # 将机器人的移动指令存入列表
        robot_moves_map[robots.index(robot)] = ' '.join(moves)
        initial_positions.append((info['good'], robot))  # 存储货物初始位置和对应的机器人初始位置

    return initial_positions, robot_moves_map

def decode_robot_moves_lists(robot_moves_map):
    robots_orders_list = []
    for rob_id, moves in robot_moves_map.items():
        rob_order_list = []
        if moves != 'get':  # 机器人能够到达货物位置
            for order in moves.split():
                if '0' <= order <= '9':
                    rob_order_list.append(f'move {rob_id} {order}')
                else:
                    rob_order_list.append(f'{order} {rob_id}')
        
        robots_orders_list.append(rob_order_list)
    return robots_orders_list

def run_orders_list(robots_orders_list, input_fun, sep='\n'):
    frames = max([len(orders) for orders in robots_orders_list])
    for i in range(frames):
        output = []
        id = input_fun()
        for ord_l in robots_orders_list:
            if ord_l:
                ord = ord_l[0]
                output.append(ord)
                ord_l.remove(ord)
        print(sep.join(output))
        print('OK')
        sys.stdout.flush()

if __name__ == '__main__':
    mymap, robots = Init_test()
    n_goods = random.randint(1, 10)  # 随机选择1到10个货物
    goods_positions = init_goods(mymap, n_goods)
    print('robots:', robots)
    initial_positions, robot_moves_map = simulate(mymap, robots, goods_positions)

    # 打印每个货物初始位置和对应的机器人初始位置
    print("Initial positions:")
    for good, robot in initial_positions:
        print(f"Good: {good}, Robot: {robot}")

    # 打印每个货物所需的机器人路径
    for i, moves_list in enumerate(robot_moves_map):
        print(f"\nMoves for good {i+1}: [{moves_list}]")

    print("initial_positions",initial_positions)
    print("robot_moves_map",robot_moves_map)
    robots_orders_list = decode_robot_moves_lists(robot_moves_map)
    print('robots_orders_list:', [len(orders) for orders in robots_orders_list])
    def Input():
        pass
    run_orders_list(robots_orders_list, Input, '\t')
