
import heapq
import sys
import random
import numpy as np
from enum import Enum
import re
import time
# import logging
import os

# 配置日志记录器
debug_file = 'D:\\大学\\0 大学竞赛\\华为云精英挑战赛\\SDK\\WindowsRelease\\sdk\\robots.log'
if os.path.exists(debug_file): os.remove(debug_file)
# logging.basicConfig(filename=debug_file, level=# logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

N = 200
robot_num = 10
berth_num = 10
boat_num = 5

class Robot:
    def __init__(self, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.x = startX
        self.y = startY
        self.carring_good = goods
        self.status = status
        self.mbx = mbx
        self.mby = mby
        self.target_good_val = 0
        self.target_berth_id = None
        self.target_berth_pos = None
    
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

def set_all_robots_target_berth_pos():  # 根据time, velocity, distance等因素为每个robot找到一个berth_pos (Init)
    def get_available_berth_pos_by_id(berth_id):  # 遍历港口4个角点，找到位于陆地上的两个点
        points = [(berth[berth_id].x, berth[berth_id].y), (berth[berth_id].x + 3, berth[berth_id].y), (berth[berth_id].x + 3, berth[berth_id].y + 3), (berth[berth_id].x, berth[berth_id].y + 3)]
        available_pos = []
        for p in points:
            if sum([mymap[(p[1] + d[1], p[0] + d[0])] == LAND for d in directions.values()]) >= 1:
                available_pos.append(p)
        return available_pos[:2]

    available_berth_pos_list = []
    for i in range(5):  # 只取前5个港口（10个点）
        available_berth_pos_list.extend(get_available_berth_pos_by_id(i))
    # # logging.info(f'available_berth_pos_list: {available_berth_pos_list}')
    rob_index = list(range(robot_num))
    for b_pos in available_berth_pos_list:
        dist_l = []
        for i in rob_index:
            rob = robot[i]
            dist_l.append([heuristic(b_pos, (rob.x, rob.y)), i])
        min_rob_i = dist_l[np.argmin(dist_l, axis=0)[0]][1]
        robot[min_rob_i].target_berth_pos = b_pos
        robot[min_rob_i].target_berth_id = get_berth_id_from_pos(b_pos)
        rob_index.remove(min_rob_i)

def Init():
    global boat_capacity
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
    sorted(berth, key=lambda b: b.transport_time)  # 排序

    set_all_robots_target_berth_pos()
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
        robot[i].carring_good, robot[i].y, robot[i].x, robot[i].status = map(int, input().split(' '))
    # logging_robot_info()
    for i in range(5):  # 5行船的信息
        boat[i].status, boat[i].target_berth_id = map(int, input().split())
    okk = input()  # 'OK'
    return id

def logging_robot_target_berth_pos_list():  #@debug
    robot_target_berth_pos_list = []
    for i in range(robot_num):
        robot_target_berth_pos_list.append(robot[i].target_berth_pos)
    # logging.info(f'robot_target_berth_pos_list: {robot_target_berth_pos_list}')

def read_line(f) -> str:  #@test
    return f.readline().strip()

def print_robot():  #@test
    print(f'robot(x, y, goods): ', end='')
    for i in range(robot_num):
        print(robot[i], end=' ')
    print()

def Init_test():  #@test
    global boat_capacity
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
    set_all_robots_target_berth_pos()
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

def heuristic(a, b) -> int:
  """
  启发式函数 - 曼哈顿距离
  """
  return abs(a[0] - b[0]) + abs(a[1] - b[1])

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

def a_star(start, goal, map, routes: list = [LAND, GOOD, BERTH], blocks: list = []):  # a*算法
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

def get_robot_path_without_overlap(rob_id, start, goal, routes: list = [LAND, GOOD, BERTH]):
    anothers_robots_path_list = []
    for i in range(robot_num):
        if i == rob_id: continue
        anothers_robots_path_list.extend(robots_path_list)
    return a_star(start, goal, mymap, routes, anothers_robots_path_list)

def get_berth_id_from_pos(pos) -> int:  # 获取当前位置对应的港口id
    for i in range(berth_num):
        b = berth[i]
        if b.x <= pos[0] < b.x + 4 and b.y <= pos[1] < b.y + 4:
            return i

def get_best_good_pos(good_list: list, cur_pos: tuple):  # 根据货物位置和价值分配（**权重可优化）
    dist_mul_val_l = []
    for i in range(len(good_list)):
        dist_mul_val_l.append(heuristic(cur_pos, (good_list[i][:2])) * good_list[i][2])
    min_i = np.argmin(dist_mul_val_l)
    gd = good_list[min_i]
    good_list.remove(gd)
    return gd

idle_berth_list = []
all_berth_pulled_good_val_list = [[]] * 5
def update_all_boats() -> list:  # 更新每艘船的状态（港口自动装载货物，装满后需要去虚拟点，到达虚拟点的船只回到空闲港口）
    def get_first_most_num_good_val_sum(num, good_val_list: list, remove=False):
        sum = 0
        num_good = len(good_val_list)
        for i in range(num):
            if i >= num_good:
                break
            sum += good_val_list[i]
        if remove:
            if i >= num_good:
                good_val_list.clear()
            else:
                for _ in range(num): good_val_list.remove(good_val_list[0])
        return sum
    
    orders_l = []
    for i in range(boat_num):
        cur_berth_id = boat[i].target_berth_id
        if boat[i].status == 1 and cur_berth_id != VIR_POINT:  # 船在港口
            good_val_list = all_berth_pulled_good_val_list[cur_berth_id]  # 当前船所在的港口上的货物
            while 1:
                if not good_val_list:  # 港口没有货物了
                    break
                v = berth[cur_berth_id].loading_speed  # 港口装载速度
                if boat[i].loaded_val + get_first_most_num_good_val_sum(v, good_val_list) > boat_capacity:  # 船装满了
                    idle_berth_list.append(cur_berth_id)  # 船离开后当前港口空闲
                    orders_l.append(f'go {i}')
                    break
                # 港口还有货物，需要装载到船上
                gd_val = get_first_most_num_good_val_sum(v, good_val_list, remove=True)
                boat[i].loaded_val += gd_val
                # logging.info(f'boat #{i} load ${gd_val}')

        elif boat[i].status == 1 and cur_berth_id == VIR_POINT:  # 船已经到达虚拟点
            boat[i].loaded_val = 0
            idle_berth_id = idle_berth_list[0]
            idle_berth_list.remove(idle_berth_id)
            orders_l.append(f'ship {i} {idle_berth_id}')  # 船前往空闲港口
    
    return orders_l  # orders_l为空时5艘船都在去虚拟点的途中

robots_path_list = [[]] * robot_num
robots_orders_list = [[]] * robot_num
num_robot_per_frame = 2  # 每一帧规划两个机器人的路径
if __name__ == "__main__":
    Init()
    
    while 1:
        frame_id = Input()
        # logging.info(f'frame #{frame_id}')

        for i in range(num_robot_per_frame):
            robot_id = (frame_id * num_robot_per_frame - (num_robot_per_frame - i)) % robot_num

            if robot[robot_id].status == 1 :  # 机器人处于正常运行状态时
                if robot[robot_id].carring_good == 0 or \
                        robot[robot_id].carring_good == 1 and not robots_orders_list[robot_id]:
                    # 机器人未携带货物|机器人携带货物并且未对其进行路径规划
                    start = (robot[robot_id].x, robot[robot_id].y)
                    if robot[robot_id].carring_good == 0:  # 机器人未携带货物
                        if good_list:  # 当生成的货物还有未分配的时
                            gd = get_best_good_pos(good_list, start)
                            goal = (gd[0], gd[1])
                            robot[robot_id].target_good_val = gd[2]
                    else:
                        goal = robot[robot_id].target_berth_pos
                    
                    path = get_robot_path_without_overlap(robot_id, start, goal)
                    robots_path_list[robot_id] = path
                
                    if not path: continue  # 机器人走不通
                    orders_l = get_orders_from_path(path, robot_id)
                    if not robots_orders_list[robot_id] or \
                            len(robots_orders_list[robot_id]) > len(orders_l):  # 新路线更短了
                        robots_orders_list[robot_id] = orders_l
                        # logging.info(f'robot #{robot_id} -> {goal}')
                    
                    else:  # 机器人处于恢复状态（发生碰撞）
                        path = []
                        # logging.info(f'robot #{robot_id} crashed!')


        output = []
        for i in range(robot_num):
            if robots_orders_list[i]:
                ord = robots_orders_list[i][0]
                output.append(ord)
                robots_orders_list[i].remove(ord)
                if not robots_orders_list[i]:  # 只剩下一条指令（下一步到达货物或港口）
                    if robot[i].carring_good == 0:  # 机器人未携带货物
                        output.append(f'get {i}')
                        # logging.info(f'robot #{i} get good at ({robot[i].x}, {robot[i].y})')
                        robot[i].carring_good = 1

                    else:  # 放货
                        output.append(f'pull {i}')
                        all_berth_pulled_good_val_list[robot[i].target_berth_id].append(robot[i].target_good_val)
                        robot[i].target_good_val = 0
                        # logging.info(f'robot #{i} pull good at ({robot[i].x}, {robot[i].y})')

        if frame_id == 1:  # 第一帧时让5艘船去前5个港口（已排序）
            for i in range(boat_num):
                output.append(f'ship {i} {i}')
        else:
            orders_l = update_all_boats()
            output.extend(orders_l)
            # logging.info(f'ship orders: {orders_l}')

        print('\n'.join(output))  # 输出当前帧的指令
        print("OK")
        sys.stdout.flush()
        if frame_id >= 15000: break

