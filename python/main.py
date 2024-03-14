
import sys
import random
import numpy as np
from enum import Enum
import re

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

robot = [Robot() for _ in range(robot_num)]

class Berth:
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.x = x
        self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed

berth = [Berth() for _ in range(berth_num)]

class Boat:
    def __init__(self, num=0, pos=0, status=0):
        self.num = num
        self.pos = pos
        self.status = status

boat = [Boat() for _ in range(boat_num)]

class RobState(Enum):
    UNREACHED = 1
    BORN_REACHED = 2
    REACHED = 3

money = 0
boat_capacity = 0
id = 0
ch = []
mymap = np.zeros((N, N))
goods = np.zeros((N, N))
berth_pos_list = []
good_pos_list = []

LAND, SEA, OBS, ROB, BERT, GOOD = 0, 1, 2, 3, 4, 5
mark = {  # 地图中的字符含义
    '.': LAND,
    '*': SEA,
    '#': OBS,
    'A': ROB,
    'B': BERT
}

directions = {  # 机器人移动方向
    '2': (0, -1), 
    '1': (-1, 0), 
    '3': (0, 1),
    '0': (1, 0)
}

def Init():
    global boat_capacity
    for i in range(N):  # 获取200x200地图
        line = input()
        chars = line.split()
        for j in range(N):
            mymap[i, j] = mark[chars[j]]

    def get_berth_pos_list_by_left_top(x, y):
        pos_l = []
        for i in range(x, x + 4):
            for j in range(y, y + 4):
                pos_l.append((i, j))
        return pos_l

    for i in range(berth_num):  # 获取泊位数据
        line = input()
        berth_list = [int(c) for c in line.split()]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        berth_pos_list.extend(get_berth_pos_list_by_left_top(berth_list[1], berth_list[2]))
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
    boat_capacity = int(input())  # 获取船的容积
    okk = input()
    print("OK")
    sys.stdout.flush()

def Init_test():  #@test
    global boat_capacity
    f_path = 'map1.txt'
    with open(f_path, 'r') as f:
        for i in range(N):  # 获取200x200地图
            line = f.readline().strip()
            chars = list(line)
            for j in range(len(chars)):
                mymap[i, j] = mark[chars[j]]
    print(f'地图初始化完成:\n{mymap}')
    
    def get_berth_pos_list_by_left_top(x, y):
        pos_l = []
        for i in range(x, x + 4):
            for j in range(y, y + 4):
                pos_l.append((i, j))
        return pos_l

    for i in range(berth_num):  # 获取泊位数据
        line = input()
        berth_list = [int(c) for c in line.split()]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        berth_pos_list.extend(get_berth_pos_list_by_left_top(berth_list[1], berth_list[2]))
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
    boat_capacity = int(input())  # 获取船的容积
    okk = input()
    print("OK")
    sys.stdout.flush()


def Input():
    id, money = map(int, input().split())  # 帧序号，当前金币数
    num = int(input())  # 新增货物的数量
    for i in range(num):  # num行新增货物的数量
        x, y, val = map(int, input().split())  # 货物坐标，金额
        # goods[x][y] = val
        good_pos_list.append((x, y))
    for i in range(robot_num):  # 10行机器人的信息
        robot[i].goods, robot[i].x, robot[i].y, robot[i].status = map(int, input().split())
    for i in range(5):  # 5行船的信息
        boat[i].status, boat[i].pos = map(int, input().split())
    okk = input()  # 'OK'
    return id

def get_all_good_pos_list(gds):
    pos_list = []
    for i in range(N):
        for j in range(N):
            if gds[i, j] != 0:
                pos_list.append((i, j))

def put_goods_into_map(gds, mp):
    gds[mp != 0] = GOOD

def rm_goods_from_map(mp, gd_pos):
    mp[gd_pos[1], gd_pos[0]] = LAND

def get_good_by_pos(x, y):
    good_pos_list.remove((x, y))

def find_way(maze, start,
                end: list,  # 终点坐标集合
                routes: list = [0],  # 机器人可以走的格子
                visited: list = [],
                orders: list = []
            ) -> [int, str]: # type: ignore
                # int:-1->机器人未到达终点，0->机器人初始位置就在终点，1->机器人下一步到达终点
    """迷宫回溯算法（非递归），返回值是单步指令和状态数组"""
    maze_h, maze_w = maze.shape
    if start in end: return RobState.BORN_REACHED, ''  # 当机器人起始位置有货物生成时
    for order in orders:
        start = (start[0] + directions[order][0], start[1] + directions[order][1])

    stack = [(start, iter(directions.items()))]
    
    while stack:
        current, directions_iter = stack[-1]
        try:
            direction, (dx, dy) = next(directions_iter)
        except StopIteration:  # 4个方向都走过了，都不满足
            stack.pop()
            direction = orders.pop()
            anti_dir = {'0': '1', '1': '0', '2': '3', '3': '2'}
            direction = anti_dir[direction]
            return RobState.UNREACHED, direction, orders, visited

        x, y = current[0] + dx, current[1] + dy
        
        if (x, y) in end:
            return RobState.REACHED, direction
        
        if 0 <= x < maze_w and 0 <= y < maze_h and maze[y, x] in routes and (x, y) not in visited:
            stack.append(((x, y), iter(directions.items())))
            visited.append((x, y))
            orders.append(direction)
            return RobState.UNREACHED, direction, orders, visited

def decode_and_append_order(orders: str, 
                            n_order: str):
    """
    向指令字符串中解析新的指令并添加其中
    `n_order`: r0 move[0-3], r0 get, r0 pull, s0 g(0)
    """
    assert re.match(r'[rs]\d\s\w+', n_order), 'Wrong order!'
    l = n_order.split(' ')
    id = int(n_order[1])
    if n_order[0] == 'r':
        if l[1] in ['get', 'pull']:
            return orders + f'{l[1]} {id}\n'
        elif re.match(r'move[0-3]', l[1]):
            return orders + f'move {id} {l[1][4]}\n'
    elif n_order[0] == 's':
        if l[1] == 'g':
            return orders + f'go {id}\n'  # 船去虚拟点
        elif re.match(r'g\d', l[1]):
            return orders + f'ship {id} {int(l[1][1])}\n'
    raise ValueError('Wrong order!')

if __name__ == "__main__":
    Init()

    rob_id = 0
    start = (robot[rob_id].x, robot[rob_id].y)
    ending = None
    args = None
    visited = []
    orders = []
    routes = [LAND, GOOD]
    target = GOOD
    output = None
    end = good_pos_list

    while 1:
        id = Input()
        # put_goods_into_map(goods, mymap)
        ending, args = find_way(mymap, start, end, routes, visited, orders)
        if ending == RobState.BORN_REACHED:
            output = decode_and_append_order(output, f'r{rob_id} get')
        elif ending == RobState.REACHED:
            output = decode_and_append_order(output, f'r{rob_id} move{args[0]}')
            if target == GOOD:
                ord = f'r{rob_id} get'
                target = BERT
                end = berth_pos_list
            else:
                ord = f'r{rob_id} pull'
                target = GOOD
                end = good_pos_list
            output = decode_and_append_order(output, ord)
        else:  # unreached
            output = decode_and_append_order(output, f'r{rob_id} move{args[0]}')
        print(output, flush=True)  # 输出当前帧的指令
        print("OK", flush=True)


        


        
    print("OK")
    sys.stdout.flush()