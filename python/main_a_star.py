
import heapq
import sys
import random
import numpy as np
from enum import Enum
import re
import time
import logging
import os

# 配置日志记录器
debug_file = 'D:\\大学\\0 大学竞赛\\华为云精英挑战赛\\SDK\\WindowsRelease\\sdk\\robots.log'
if os.path.exists(debug_file): os.remove(debug_file)
logging.basicConfig(filename=debug_file, level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

N = 200
robot_num = 10
berth_num = 10
boat_num = 5

class Robot:
    def __init__(self, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.x = startX
        self.y = startY
        self.goods = goods
        self.status = status
        self.mbx = mbx
        self.mby = mby
        self.target_good_val = 0

    def __str__(self) -> str:
        return f'({self.x}, {self.y}, {self.goods})'
robot = [Robot() for _ in range(robot_num)]

class Berth:
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.x = x
        self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed
berth = [Berth() for _ in range(berth_num)]

class Boat:
    def __init__(self, target_berth_id=0, status=0):
        self.target_berth_id = target_berth_id
        self.status = status
        self.loaded_val = 0

boat = [Boat() for _ in range(boat_num)]

class RobState(Enum):
    UNREACHED = 1
    BORN_REACHED = 2
    REACHED = 3

money = 0
boat_capacity = 0
frame_id = 0
ch = []
mymap = np.zeros((N, N))
# goods = np.zeros((N, N))
good_list = []

LAND, SEA, OBS, ROB, BERTH, GOOD, VIR_POINT = 0, 1, 2, 3, 4, 5, -1
mark = {  # 地图中的字符含义
    '.': LAND,
    '*': SEA,
    '#': OBS,
    'A': ROB,
    'B': BERTH,
    '$': GOOD
}

reverse_mark = {}
for k, v in mark.items():
    reverse_mark[v] = k

directions = {  # 机器人移动方向
    '2': (0, -1), 
    '1': (-1, 0), 
    '3': (0, 1),
    '0': (1, 0)
}
reverse_directions = {}
for k, v in directions.items():
    reverse_directions[v] = k

def get_berth_pos_list_by_left_top(x, y):
    pos_l = []
    for i in range(x, x + 4):
        for j in range(y, y + 4):
            pos_l.append((i, j))
    return pos_l

def get_the_best_berth_id():  # 根据time, velocity, distance等因素找到最合适的一个港口
    return 0

target_berth = None
target_berth_pos = None
def Init():
    global boat_capacity, target_berth, target_berth_pos
    for i in range(N):  # 获取200x200地图
        line = input()
        ch.append(line.strip())
    for i in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(' ')]
        id = berth_list[0]
        berth[id].y = berth_list[1]
        berth[id].x = berth_list[2]
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
    boat_capacity = int(input())
    okk = input()
    
    for i in range(N):
        for j in range(N):
            mymap[i, j] = mark[ch[i][j]]
    target_berth = get_the_best_berth_id()
    target_berth_pos = (berth[target_berth].x + 3, berth[target_berth].y + 3)
    print("OK")
    sys.stdout.flush()

def Input():
    id, money = map(int, input().split(' '))  # 帧序号，当前金币数
    num = int(input())  # 新增货物的数量
    for i in range(num):  # num行新增货物的数量
        y, x, val = map(int, input().split(' '))  # 货物坐标，金额
        # goods[x][y] = val
        good_list.append((x, y, val))
    for i in range(robot_num):  # 10行机器人的信息
        robot[i].goods, robot[i].y, robot[i].x, robot[i].status = map(int, input().split(' '))
    # logging_robot_info()
    for i in range(5):  # 5行船的信息
        boat[i].status, boat[i].target_berth_id = map(int, input().split())
    okk = input()  # 'OK'
    return id

def logging_robot_info():  #@debug
    robots_pos = []
    for i in range(robot_num):
        robots_pos.append((robot[i].x, robot[i].y))
    logging.info(f'robots(x, y): {robots_pos}')

def read_line(f) -> str:  #@test
    return f.readline().strip()

def print_robot():  #@test
    print(f'robot(x, y, goods): ', end='')
    for i in range(robot_num):
        print(robot[i], end=' ')
    print()

def Init_test():  #@test
    global boat_capacity, target_berth, target_berth_pos
    f_path = 'D:\\大学\\0 大学竞赛\\华为云精英挑战赛\\SDK\\WindowsRelease\\sdk\\debug.txt'
    f = open(f_path, 'r')
    rob_i = 0
    for i in range(N):  # 获取200x200地图
        line = read_line(f).strip()
        for j in range(N):
            mymap[i, j] = mark[line[j]]
            if mymap[i, j] == ROB:
                robot[rob_i].x = j
                robot[rob_i].y = i
                rob_i += 1
    print(f'地图初始化完成:\n{mymap}')
    print_robot()

    def get_berth_pos_list_by_left_top(x, y):
        pos_l = []
        for i in range(x, x + 4):
            for j in range(y, y + 4):
                pos_l.append((i, j))
        return pos_l

    for i in range(berth_num):  # 获取泊位数据
        line = f.readline().strip()
        # print('line', line)
        berth_list = [int(c) for c in line.split()]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        # berth_pos_list.extend(get_berth_pos_list_by_left_top(berth_list[1], berth_list[2]))
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
    print(f'泊位初始化完成...')
    boat_capacity = int(f.readline().strip())  # 获取船的容积
    print(f'船初始化完成，容量：{boat_capacity}')
    target_berth = get_the_best_berth_id()
    target_berth_pos = (berth[target_berth].x + 3, berth[target_berth].y + 3)
    print("初始化完成...")
    return f

def Input_test(f):  #@test
    id = int(read_line(f))  # frame id
    num = int(read_line(f))  # 新增货物的数量
    for i in range(num):  # num行新增货物的数量
        x, y, val = map(int, read_line(f).split())  # 货物坐标，金额
        # goods[x][y] = val
        # mymap[y][x] = GOOD
        good_list.append((x, y, val))
    refresh_map_by_order(mymap)
    return id

def move_robot(rob_i, order):  #@test
    rob = robot[rob_i]
    mymap[rob.y, rob.x] = LAND
    dx, dy = directions[order]
    rob.x += dx
    rob.y += dy
    mymap[rob.y, rob.x] = ROB

def refresh_map_by_order(mymap,  #@test
                         order: str = '',  # move
                         map_file='map_with_goods.txt'):
    
    if order != '':
        print(f'order: {order}')
        if order[:4] == 'move':
            move_robot(int(order[5]), order[7])

    map_str = []
    for i in range(N):
        line = ''
        for j in range(N):
            line += reverse_mark[mymap[i, j]]
        map_str.append(line)
    for gd in good_list:
        gd_pos = (gd[0], gd[1])
        s_l = list(map_str[gd_pos[1]])
        s_l[gd_pos[0]] = reverse_mark[GOOD]
        map_str[gd_pos[1]] = ''.join(s_l)
    with open(map_file, "w") as file:
        # 遍历字符串数组
        for string in map_str:
            # 写入每个字符串并添加换行符
            file.write(string + "\n")

def heuristic(a, b):
  """
  启发式函数 - 曼哈顿距离
  """
  return abs(a[0] - b[0]) + abs(a[1] - b[1])

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

def reconstruct_path(came_from, current):  # 路径重建
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]

def get_orders_from_path(path, rob_id):
    orders = []
    for i in range(len(path) - 1):
        direct = reverse_directions[tuple(np.array(path[i + 1]) - np.array(path[i]))]
        orders.append(f'move {rob_id} {direct}')
    return orders

def a_star(start, goal, map, routes: list = [LAND, GOOD, BERTH]):  # A* 算法实现
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
            if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height and map[neighbor[1], neighbor[0]] in routes:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    if neighbor not in [i[2] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], tentative_g_score, neighbor))
    return []

def get_berth_id_from_pos(pos):
    # 获取当前位置对应的港口id
    for i in range(berth_num):
        b = berth[i]
        if b.x <= pos[0] < b.x + 4 and b.y <= pos[1] < b.y + 4:
            return i

def get_nearest_good_pos(good_list: list, cur_pos: tuple):  # 根据机器人的位置分配距离最近的货物
    gd = good_list.pop()
    return gd

def get_nearest_berth_pos(cur_pos):  # 离机器人当前位置最近的港口
    dist_heur = np.zeros(berth_num)
    for i in range(berth_num):
        dist_heur[i] = heuristic(cur_pos, (berth[i].x, berth[i].y))
    min_bert_id = np.argmin(dist_heur)
    return (berth[min_bert_id].x + 3, berth[min_bert_id].y + 3)

def update_all_boats(gd_val_list) -> str:  # 更新每艘船的状态（港口自动装载货物，装满后需要去虚拟点，并且对到达虚拟点的船只召回到港口）
    for i in range(boat_num):
        if boat[i].status == 1 and boat[i].target_berth_id == target_berth:  # 船在港口
            while 1:
                if not gd_val_list:  # 港口没有货物了
                    break
                if boat[i].loaded_val + gd_val_list[-1] > boat_capacity:  # 船装满了
                    return f'go {i}'
                gd_val = gd_val_list.pop()
                boat[i].loaded_val += gd_val
                logging.info(f'boat #{i} load ${gd_val}')

        elif boat[i].status == 1 and boat[i].target_berth_id == VIR_POINT:  # 船已经到达虚拟点
            boat[i].loaded_val = 0
            # boats_targets_list[i] = target_berth
            return f'ship {i} {target_berth}'
    return ''  # 5艘船都在去虚拟点的途中


robots_orders_list = [[]] * robot_num
num_robot_per_frame = 2  # 每一帧规划两个机器人的路径
pulled_good_val_list = []

if __name__ == "__main__":
    Init()
    # f = Init_test()

    for zhen in range(1, 15001):
        frame_id = Input()
        # frame_id = Input_test(f)
        # print(f'frame #{frame_id}')

        logging.info(f'frame #{frame_id}')

        for i in range(num_robot_per_frame):
            robot_id = (frame_id * num_robot_per_frame - (num_robot_per_frame - i)) % robot_num
            start = (robot[robot_id].x, robot[robot_id].y)
            if robot[robot_id].goods == 0:  # 机器人未携带货物
                if good_list:  # 当生成的货物还有未分配的时
                    gd = get_nearest_good_pos(good_list, start)
                    goal = (gd[0], gd[1])
                    robot[robot_id].target_good_val = gd[2]
            else:
                goal = target_berth_pos
            path = a_star(start, goal, mymap)
            if not path:
                continue  # 机器人走不通
            orders_l = get_orders_from_path(path, robot_id)
            if not robots_orders_list[robot_id] or \
                    len(robots_orders_list[robot_id]) > len(orders_l):
                robots_orders_list[robot_id] = orders_l
                logging.info(f'robot #{robot_id} -> {goal}')

        output = []
        for i in range(robot_num):
            if robots_orders_list[i]:
                ord = robots_orders_list[i][0]
                output.append(ord)
                robots_orders_list[i].remove(ord)
                if not robots_orders_list[i]:  # 只剩下一条指令（下一步到达货物或港口）
                    if robot[i].goods == 0:  # 机器人未携带货物
                        output.append(f'get {i}')
                        logging.info(f'robot #{i} get good at ({robot[i].x}, {robot[i].y})')

                    else:  # 放货
                        output.append(f'pull {i}')
                        pulled_good_val_list.append(robot[i].target_good_val)
                        robot[i].target_good_val = 0
                        # robots_targets_list[i] = GOOD
                        logging.info(f'robot #{i} pull good at ({robot[i].x}, {robot[i].y})')

        if frame_id == 1:  # 第一帧时让所有船前往target_berth
            for i in range(boat_num):
                output.append(f'ship {i} {target_berth}')
        else:
            order = update_all_boats(pulled_good_val_list)
            if order != '':
                output.append(order)
                logging.info(f'ship order: {order}')

        print('\n'.join(output))  # 输出当前帧的指令
        print("OK")
        sys.stdout.flush()
                
        # for order in output:
        #     refresh_map_by_order(mymap, order)
            
